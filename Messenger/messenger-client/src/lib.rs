use std::time::Duration;

use bytes::{Buf, BufMut, Bytes, BytesMut};
use tokio::{
    io::AsyncWriteExt,
    net::{tcp::OwnedWriteHalf, TcpStream},
    sync::mpsc::{self, UnboundedReceiver, UnboundedSender},
    task::JoinHandle,
};
use tokio_stream::StreamExt;
use tokio_util::codec::{Decoder, FramedRead};

#[derive(Debug)]
pub struct Message {
    pub name: String,
    pub data: Bytes,
}

struct MessageDecoder;
impl Decoder for MessageDecoder {
    type Item = Message;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if src.len() < 2 {
            // Have not received type length prefix yet
            return Ok(None);
        }

        let mut name_len_bytes = [0u8; 2];
        name_len_bytes.copy_from_slice(&src[..2]);
        let name_len = u16::from_be_bytes(name_len_bytes) as usize;

        if src.len() < 2 + name_len + 4 {
            // Have not received name and data length yet
            src.reserve(2 + name_len + 4 - src.len());
            return Ok(None);
        }

        let mut data_len_bytes = [0u8; 4];
        data_len_bytes.copy_from_slice(&src[(2 + name_len)..(6 + name_len)]);
        let data_len = i32::from_be_bytes(data_len_bytes) as usize;

        if src.len() < 6 + name_len + data_len {
            // Have not received data yet
            src.reserve(6 + name_len + data_len - src.len());
            return Ok(None);
        }

        // If we reach here, we have the full message data

        let name_data = src[2..2 + name_len].to_vec();
        let data = BytesMut::from(&src[6 + name_len..6 + name_len + data_len]);
        src.advance(6 + name_len + data_len);

        let name = match String::from_utf8(name_data) {
            Ok(string) => Ok(string),
            Err(decode_err) => Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                decode_err,
            )),
        }?;

        Ok(Some(Message {
            name,
            data: data.into(),
        }))
    }
}

enum ToIoCmd {
    Listen(String),
    SendMessage(Message),
    Disconnect,
}

fn pack_str(string: &str, buf: &mut BytesMut) {
    buf.put_u16(string.len() as u16);
    buf.put(string.as_bytes());
}

fn pack_message(m: Message) -> BytesMut {
    let mut dst = BytesMut::with_capacity(6 + m.name.len() + m.data.len());
    pack_str(&m.name, &mut dst);
    dst.put_i32(m.data.len() as i32);
    dst.put(m.data);
    dst
}

async fn send_listen_message(
    write_half: &mut OwnedWriteHalf,
    name: &str,
) -> Result<(), std::io::Error> {
    let mut data = BytesMut::with_capacity(name.len() + 2);
    pack_str(&name, &mut data);
    write_half
        .write_all(&pack_message(Message {
            name: "_Listen".to_string(),
            data: data.into(),
        }))
        .await
}

async fn message_io(
    addr: &str,
    name: &str,
    in_send: &mpsc::UnboundedSender<Message>,
    out_recv: &mut mpsc::UnboundedReceiver<ToIoCmd>,
    listening: &mut Vec<String>,
) -> Result<(), std::io::Error> {
    let stream = TcpStream::connect(addr).await?;
    let (read_half, mut write_half) = stream.into_split();

    // Send client name
    let mut name_buf = BytesMut::with_capacity(2 + name.len());
    pack_str(name, &mut name_buf);
    write_half.write_all(&name_buf).await?;

    println!("Messenger connected");

    for name in listening {
        send_listen_message(&mut write_half, &name).await?;
    }

    let mut read = FramedRead::new(read_half, MessageDecoder);

    let mut heartbeat_interval = tokio::time::interval(Duration::from_secs(1));
    let packed_heartbeat = pack_message(Message {
        name: "_Heartbeat".to_string(),
        data: Bytes::new(),
    });

    loop {
        tokio::select! {
            _ = heartbeat_interval.tick() => {
                write_half.write_all(&packed_heartbeat).await?;
            }

            result = out_recv.recv() => {match result {
                Some(cmd) => match cmd {
                    ToIoCmd::Listen(name) => {
                        send_listen_message(&mut write_half, &name).await?;
                    }
                    ToIoCmd::SendMessage(msg) => {
                        write_half.write_all(&pack_message(msg)).await?;
                    }
                    ToIoCmd::Disconnect => break
                }
                None => break
            }}

            result = read.next() => match result {
                Some(res) => {
                    let msg = res?;
                    if msg.name != "_Heartbeat" {
                        if let Err(_) = in_send.send(msg) {
                            break
                        }
                    }
                }
                None => return Err(std::io::Error::from(std::io::ErrorKind::BrokenPipe))
            }
        }
    }

    write_half
        .write_all(&pack_message(Message {
            name: "_Disconnect".to_string(),
            data: Bytes::new(),
        }))
        .await?;

    Ok(())
}

pub struct MessengerClient {
    io_join: JoinHandle<()>,
    in_recv: UnboundedReceiver<Message>,
    out_send: UnboundedSender<ToIoCmd>,
}

impl MessengerClient {
    pub fn new(addr: String, name: String) -> Self {
        let (in_send, in_recv) = mpsc::unbounded_channel();
        let (out_send, mut out_recv) = mpsc::unbounded_channel();
        let io_join = tokio::spawn(async move {
            let mut listening = vec![];

            loop {
                match message_io(&addr, &name, &in_send, &mut out_recv, &mut listening).await {
                    // Ok indicates client disconnected cleanly
                    Ok(_) => break,
                    Err(e) => println!("Messenger connection failed: {}", e),
                }

                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });

        Self {
            io_join,
            in_recv,
            out_send,
        }
    }

    pub fn listen(&mut self, name: &str) {
        let _ = self.out_send.send(ToIoCmd::Listen(name.to_string()));
    }

    pub fn listen_multiple(&mut self, names: Vec<&str>) {
        for name in names {
            self.listen(name);
        }
    }

    pub fn send_message(&mut self, msg: Message) {
        let _ = self.out_send.send(ToIoCmd::SendMessage(msg));
    }

    pub fn send_empty(&mut self, name: &str) {
        self.send_message(Message {
            name: name.to_string(),
            data: Bytes::new(),
        });
    }

    pub fn poll_recv_message(&mut self) -> Option<Message> {
        self.in_recv.try_recv().ok()
    }

    /// Gets all available incoming messages immediately if any are available,
    /// otherwise waits for one to arrive
    pub async fn poll_or_await_messages(&mut self) -> Vec<Message> {
        match self.poll_recv_message() {
            Some(msg) => {
                let mut messages = vec![msg];
                loop {
                    match self.poll_recv_message() {
                        Some(m) => messages.push(m),
                        None => return messages,
                    }
                }
            }
            None => match self.in_recv.recv().await {
                Some(m) => vec![m],
                None => vec![],
            },
        }
    }

    pub async fn disconnect(self) {
        match self.out_send.send(ToIoCmd::Disconnect) {
            Ok(_) => {
                let _ = self.io_join.await;
            }
            Err(_) => {}
        };
    }
}
