use std::time::Duration;

use bytes::{Buf, BufMut, Bytes, BytesMut};
use tokio::{
    io::AsyncWriteExt,
    net::TcpStream,
    sync::mpsc::{self, UnboundedReceiver, UnboundedSender},
    task::JoinHandle,
};
use tokio_stream::StreamExt;
use tokio_util::codec::{Decoder, FramedRead};

#[derive(Debug)]
pub struct Message {
    name: String,
    data: Bytes,
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

async fn message_io(
    addr: &str,
    name: &str,
    in_send: &mpsc::UnboundedSender<Message>,
    out_recv: &mut mpsc::UnboundedReceiver<ToIoCmd>,
) -> Result<(), std::io::Error> {
    let stream = TcpStream::connect(addr).await?;
    let (read_half, mut write_half) = stream.into_split();

    // Send client name
    let mut name_buf = BytesMut::with_capacity(2 + name.len());
    pack_str(name, &mut name_buf);
    write_half.write_all(&name_buf).await?;

    let mut read = FramedRead::new(read_half, MessageDecoder);

    let mut heartbeat_interval = tokio::time::interval(Duration::from_secs(1));
    let packed_heartbeat = pack_message(Message {
        name: "_Heartbeat".to_string(),
        data: Bytes::new(),
    });

    loop {
        tokio::select! {
            _ = heartbeat_interval.tick() => {
                println!("Writing heartbeat");
                write_half.write_all(&packed_heartbeat).await?;
            }

            result = out_recv.recv() => {match result {
                Some(cmd) => match cmd {
                    ToIoCmd::SendMessage(msg) => {
                        println!("Writing message: {:?}", msg);
                        write_half.write_all(&pack_message(msg)).await?;
                    }
                    ToIoCmd::Disconnect => break
                }
                None => break
            }}

            result = read.next() => match result {
                Some(res) => {
                    println!("Read: {:?}", res);
                    if let Err(_) = in_send.send(res?) {
                        break
                    }
                }
                None => return Err(std::io::Error::from(std::io::ErrorKind::BrokenPipe))
            }
        }
    }

    println!("Disconnecting");
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
    pub async fn new(addr: String, name: String) -> Self {
        let (in_send, in_recv) = mpsc::unbounded_channel();
        let (out_send, mut out_recv) = mpsc::unbounded_channel();
        let io_join = tokio::spawn(async move {
            loop {
                match message_io(&addr, &name, &in_send, &mut out_recv).await {
                    // Ok indicates client disconnected cleanly
                    Ok(_) => break,
                    Err(e) => println!("Connection failed: {}", e),
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

    pub fn send_message(&mut self, msg: Message) {
        let _ = self.out_send.send(ToIoCmd::SendMessage(msg));
    }

    pub fn poll_recv_message(&mut self) -> Option<Message> {
        self.in_recv.try_recv().ok()
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
