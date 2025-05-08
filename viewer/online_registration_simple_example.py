'''
Start stream for VI, SI and AHAT (or LT)
Call pcd registration function within online stream.
'''

import cv2
import hl2ss
import multiprocessing as mp
import hl2ss_mp
from pynput import keyboard
from pathlib import Path
from integration_function import volume_integration

#-------------------------------------------------------------------------------------------------------------------------
# CONFIG

# Import registration method
# from online_registration_eye import register           #==> Eye ransac init
from online_registration_manual import register       #==> Manual init

# IP of HoloLens
host = '10.171.51.181'

acq_name = 'XR_day'
acq_nr = 0

# Voice command list
strings = ['align',     # 0
           'spine',     # 1
           'abdomen']   # 2

# Define starting model
model = 3

# 3 = Spine
# 5 = Abdomen

#-------------------------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------------------------
# DATA ACQUISITION

frames_to_send = 50
acquire_frames = 50
eyes_to_send = 100

#-------------------------------------------------------------------------------------------------------------------------
# STREAM CONFIG
# Ports
ports = [
    # hl2ss.StreamPort.RM_DEPTH_AHAT,
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_DEPTH_LONGTHROW,
    hl2ss.StreamPort.SPATIAL_INPUT
    ]

port_vi = hl2ss.IPCPort.VOICE_INPUT

if hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports:
    lt = True
else:
    lt = False

# RM Depth AHAT parameters
ahat_mode = hl2ss.StreamMode.MODE_1
ahat_profile = hl2ss.VideoProfile.H265_MAIN
ahat_bitrate = 8*1024*1024

# RM Depth Long Throw parameters
lt_mode = hl2ss.StreamMode.MODE_1
lt_filter = hl2ss.PngFilterMode.Paeth

# RM VLC parameters
vlc_mode    = hl2ss.StreamMode.MODE_1
vlc_profile = hl2ss.VideoProfile.H265_MAIN
vlc_bitrate = 1*1024*1024

# Maximum number of frames in buffer
buffer_elements = 60
keep_frames = 500

#-------------------------------------------------------------------------------------------------------------------------
# START STREAM

if __name__ == '__main__':

    producer = hl2ss_mp.producer()
    producer.configure_rm_vlc(True, host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.ChunkSize.RM_VLC, vlc_mode, vlc_profile, vlc_bitrate)
    producer.configure_rm_depth_ahat(True, host, hl2ss.StreamPort.RM_DEPTH_AHAT, hl2ss.ChunkSize.RM_DEPTH_AHAT, ahat_mode, ahat_profile, ahat_bitrate)
    producer.configure_rm_depth_longthrow(True, host, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, hl2ss.ChunkSize.RM_DEPTH_LONGTHROW, lt_mode, lt_filter)
    producer.configure_si(host, hl2ss.StreamPort.SPATIAL_INPUT, hl2ss.ChunkSize.SPATIAL_INPUT)

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)

    manager = mp.Manager()
    consumer = hl2ss_mp.consumer()
    sinks = {}

    for port in ports:
        sinks[port] = consumer.create_sink(producer, port, manager, None)
        sinks[port].get_attach_response()

    # Functions for handeling data packages
    def display_basic(port, payload):
        cv2.imshow(hl2ss.get_port_name(port), payload)

    def display_depth(port, payload):
        cv2.imshow(hl2ss.get_port_name(port) + '-depth', payload.depth * 8) # Scaled for visibility

    def get_word(strings, index):
        if ((index < 0) or (index >= len(strings))):
            return '_UNKNOWN_'
        else:
            return strings[index]


    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    client_vi = hl2ss.ipc_vi(host, port_vi)
    client_vi.open()

    client_vi.create_recognizer()
    if (client_vi.register_commands(True, strings)):
        print('Ready. Say any of the follow commands:')
        print(*strings, sep=", ")
        client_vi.start()

        data_depth = []
        data_grey = []
        data_si = []

        data_depth_option = []
        data_grey_option = []

        while (enable):
            for port in ports:
                f, data = sinks[port].get_most_recent_frame()
                if (f >= 0):
                    sinks[port].get_buffered_frame(f)
                
                # Process AHAT stream
                if (port == hl2ss.StreamPort.RM_DEPTH_AHAT):
                    if (data is not None):
                        
                        # Display depth frame
                        display_depth(port, data.payload)

                        # Save depth data packs to memory
                        data_depth.append(data)
                        
                        # Keep only the recent frames
                        del data_depth[:-keep_frames]

                # Process LT stream
                if (port == hl2ss.StreamPort.RM_DEPTH_LONGTHROW):
                    if (data is not None):
                        
                        # Display depth frame
                        display_depth(port, data.payload)

                        # Save depth data packs to memory
                        data_depth.append(data)

                        # Keep only the recent frames
                        del data_depth[:-keep_frames]
                
                # Process greyscale stream
                if (port == hl2ss.StreamPort.RM_VLC_LEFTFRONT):
                    if (data is not None):
                        # Display greyscale frame
                        display_basic(port, data.payload)

                        # Save depth data packs to memory
                        data_grey.append(data)

                        # Keep only the recent frames
                        del data_grey[:-keep_frames]

                # Process spatial input stream
                if (port == hl2ss.StreamPort.SPATIAL_INPUT):
                    if (data is not None):
                        # Save spatial input data packs to memory
                        data_si.append(data)

                        # Keep only the recent poses
                        del data_si[:-keep_frames]

            # Listen for VI events            
            events = client_vi.pop()
            for event in events:
                event.unpack()
                print(f'Event: {get_word(strings, event.index)} {event.index} {event.confidence} {event.phrase_duration} {event.phrase_start_time} {event.raw_confidence}')

                # Run registration script if voice input event
                if event.index == 0: # align
                    try:
                        save_folder = Path.cwd() / 'data' / 'demo_registrations' / f'registration_{model}_{acq_name}_{acq_nr}'

                        if not save_folder.is_dir(): save_folder.mkdir()

                        target_pcd = volume_integration(data_depth[-frames_to_send:], data_grey[-frames_to_send:], lt=lt, save_folder=save_folder)
                        register(host, model, target_pcd, data_si[-eyes_to_send:], save_folder=save_folder)

                        acq_nr += 1

                    except Exception as e: print(e)
                
                # Voice commands to switch between models
                if event.index == 1: # spine
                    model = 3
                
                if event.index == 2: # abdomen
                    model = 5
                 
            cv2.waitKey(1)
                    

        for port in ports:
            sinks[port].detach()

        for port in ports:
            producer.stop(port)
        
        client_vi.stop()
        client_vi.clear()

    client_vi.close()

    listener.join()
