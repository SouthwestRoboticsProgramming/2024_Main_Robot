import pretty_midi
import math
from dataclasses import dataclass

highest_note = 79 # G5
lowest_note = 24 # C1

# How many times to duplicate the note
def get_weight(note):
    return 1

track_id = 0
class Track:
    def __init__(self):
        global track_id
        self.id = track_id
        track_id += 1
        self.notes = []
        self.timestamps = []
        self.down_note = None
        self.down_count = 0

    def note_down(self, time, note):
        # print("track", self.id, "note down", note)
        if self.down_note != note:
            self.notes.append(note)
            self.timestamps.append(time)
            self.down_note = note
        self.down_count += 1

    def note_up(self, time):
        # print("track", self.id, "note up", self.down_note)
        self.down_count -= 1
        if self.down_count == 0:
            self.timestamps.append(time)
            self.down_note = None

    def is_playing(self, note):
        return self.down_note == note

    def is_idle(self):
        return self.down_note == None

    def can_play(self, note):
        # Play if already playing this note or not playing
        return self.down_note == note or self.down_note == None

class TrackSplitter:
    def __init__(self):
        self.tracks = []

    def print_tracks(self, prefix):
        line = prefix + " | "
        for track in self.tracks:
            if track.down_note == None:
                line += "------"
            else:
                line += f"{track.down_note:3}:{track.down_count:2}"
            line += " "
        print(line)

    def note_down(self, time, note):
        weight = get_weight(note)

        tracks = []

        # First search for tracks already playing the note
        for possible_track in self.tracks:
            if possible_track.is_playing(note):
                tracks.append(possible_track)
                if len(tracks) >= weight:
                    break

        # Then search for idle tracks
        for possible_track in self.tracks:
            if len(tracks) >= weight:
                break
            if possible_track.is_idle():
                tracks.append(possible_track)

        # If none can play the note now, make new ones
        while len(tracks) < weight:
            track = Track()
            tracks.append(track)
            self.tracks.append(track)

        for track in tracks:
            track.note_down(time, note)

        # self.print_tracks("note down: " + str(note))

    def note_up(self, time, note):
        to_remove = get_weight(note)

        # Find the tracks that are playing this note
        for track in reversed(self.tracks):
            if track.is_playing(note):
                track.note_up(time)
                to_remove -= 1
                if to_remove == 0:
                    break

        if to_remove != 0:
            print("Somehow a note ended more than it started:", note)

        # self.print_tracks("note up:   " + str(note))

@dataclass
class NoteEdge:
    action: str # "up" or "down"
    note: int
    timestamp: float

def convert_midi(midi_file):
    # ---- Find edges
    midi = pretty_midi.PrettyMIDI(midi_file)

    edges = []
    for inst in midi.instruments:
        if inst.is_drum:
            print("Skipping drum track")
            continue

        max_note = 0
        for note in inst.notes:
            max_note = max(max_note, note.pitch)

        # Shift whole track down if it goes too high
        if max_note <= highest_note:
            shift = 0
        else:
            amount_over = max_note - highest_note
            octaves = math.ceil(amount_over / 12)
            shift = 12 * octaves

        for note in inst.notes:
            # Move up if too low
            pitch = note.pitch - shift
            if pitch < lowest_note:
                amount_under = lowest_note - pitch
                octaves = math.ceil(amount_under / 12)
                pitch += 12 * octaves

            edges.append(NoteEdge("down", pitch, note.start))
            edges.append(NoteEdge("up", pitch, note.end))
    print("Song has", len(edges), "edges")

    # Put them all in order
    print("Sorting edges")
    edges.sort(key=lambda edge: edge.timestamp)

    # ---- Split into tracks
    print("Splitting tracks")
    splitter = TrackSplitter()
    for edge in edges:
        if edge.action == "down":
            splitter.note_down(edge.timestamp, edge.note)
        elif edge.action == "up":
            splitter.note_up(edge.timestamp, edge.note)
        else:
            print("what")
    raw_tracks = splitter.tracks
    print("Song needed", len(raw_tracks), "tracks")
    
    # ---- Repeat tracks to use all motors
    motor_count = 14
    inst_count = len(raw_tracks)

    tracks = []
    which = 0
    repeat_count = 0
    for _ in range(motor_count):
        tracks.append(raw_tracks[which])
        which += 1
        if which >= inst_count:
            which = 0
            repeat_count += 1
    
    print("Repeating each track", repeat_count, "times")

    # ---- Chirpify the tracks
    print("Generating CHRP")
    chirp = b"".join([
        int("0x01000000", 16).to_bytes(4, byteorder="little"),
        len(tracks).to_bytes(2, byteorder="little"),
        b"".join(
            len(track.notes).to_bytes(2, byteorder="little") for track in tracks
        ),
        note_datas := b"".join(
            b"".join([
                int((440 / 32) * 2 ** ((track.notes[note_idx] - 9) / 12)).to_bytes(
                    2, byteorder="little"
                ),
                (0xFFFF).to_bytes(2, byteorder="little"), # LOUD
                int(track.timestamps[note_idx * 2] * 1000).to_bytes(4, byteorder="little"), # Start
                int(track.timestamps[note_idx*2+1] * 1000).to_bytes(4, byteorder="little"), # End
            ])
            for track in tracks
            for note_idx in range(len(track.notes))
        ),
        (
            2**32
            - 1
            - len(tracks) % 256
            - len(tracks) // 256
            - sum(len(track.notes) % 256 for track in tracks)
            - sum(len(track.notes) // 256 for track in tracks)
            - sum(note_datas)
        ).to_bytes(4, byteorder="little"),
    ])
    return chirp



from pathlib import Path
import sys
import os

# Usage: python midi2chirp.py inputfile.midi outputfile.chrp
# Path(sys.argv[2]).write_bytes(convert_midi(sys.argv[1]))

for filename in os.listdir(sys.argv[1]):
    if not filename.endswith(".mid") and not filename.endswith(".midi"):
        continue
    
    path = os.path.join(sys.argv[1], filename)
    try:
        chrp_data = convert_midi(path)
    except:
        print(filename, "failed")
        continue
    chrp_path = os.path.join(sys.argv[2], filename) + ".chrp"

    Path(chrp_path).write_bytes(chrp_data)
    print("Converted", path, "to", chrp_path)
    
    os.system("scp \"" + chrp_path + "\" admin@10.21.29.2:/home/lvuser/deploy/music/")
    print("Deployed it")
