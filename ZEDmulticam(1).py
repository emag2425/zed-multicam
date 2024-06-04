import os
import sys
from signal import signal, SIGINT
import time
from datetime import datetime
import threading
import csv
import tkinter as tk
from tkinter import messagebox
import cv2
import pyzed.sl as sl 
from PIL import Image, ImageTk

# ========== Global Variable ========== #
zed_list = []  # Camera array
left_list = []

class ZEDRecording:
    is_recording = False
    def __init__(self, master, export_path):
        self.master = master
        master.title('ZED Recording GUI')
       
        self.is_recording = False
        
        self.name_list = []


        self.timestamp_list = []
        self.last_ts_list = []
        self.last_frames = []
        
        self.n_frames = []  # Tracks number of recorded frames 
        self.dropped_count = []  #dropped frames, index per camera
        self.stop_time = 0
        self.start_time = 0

        # Set recordign path
        self.rec_path = export_path
        self.datefolder = time.strftime("%Y-%m-%d")  #folder for the days testing
        self.rec_path = "/".join([self.rec_path,self.datefolder])
        os.makedirs(self.rec_path, exist_ok=True)  #create folder to save to

        
        # Set Init Parameters
        print("\n[INFO] SETTING INIT PARAMETERS")
        init_params = sl.InitParameters(
            camera_resolution=sl.RESOLUTION.AUTO,
            camera_fps=30,
            depth_mode=sl.DEPTH_MODE.PERFORMANCE,
        )

        # Obtain available devices' serial numbers
        available_devices = sl.Camera.get_device_list()
        
        #populate zed_list, left_list, name_list
        index = 0
        for dev in available_devices:
            init_params.set_from_serial_number(dev.serial_number)
            self.name_list.append("ZED {}".format(dev.serial_number))
            zed_list.append(sl.Camera())
            left_list.append(sl.Mat())
            self.timestamp_list.append(0)
            self.last_ts_list.append(0)
            self.last_frames.append(None)
            self.n_frames.append(0)
            self.dropped_count.append(0)
            openerr = zed_list[index].open(init_params)
            if openerr != sl.ERROR_CODE.SUCCESS:
                print(repr(openerr))
                zed_list[index].close()
            index = index + 1        

        self.recstat_lbl = None  # Recording status label

        self.metadata = [
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
        ]
        self.metadata_fname = ""

        #TK variables
        self.status = tk.StringVar()  # Store the status of recording
        self.status.set("Recording Stopped")  # Initially set to stopped
        self.plot = tk.IntVar()  # Plot
        self.plot.set(0)
        self.row = tk.IntVar()  # Row
        self.row.set(0)        
        self.expAuto = tk.StringVar()  #boolean is camera in auto-exposure mode
        self.expAuto.set("Auto Exp")
        self.exposure = tk.IntVar()  
        self.exposure.set(21565) #init value micro seconds
        #self.GPS = tk.StringVar()
        #self.GPS.set("GPS not connected")  #default status for app open
        self.RTKStat = tk.IntVar()
        self.RTKStat.set(0)

        # Label
        self.plot_lbl = tk.Label(
            master,
            text="Plot number",
            font=("Times New Roman", 14),
             bg="gray69",
        )
        self.plot_lbl.grid(row=3, column=0, padx=(15, 0), pady=(15, 15), sticky="E")
        self.row_lbl = tk.Label(
            master,
           text="Row number",
            font=("Times New Roman", 14),
            bg="gray69",
        )
        self.row_lbl.grid(row=4, column=0, padx=(15, 0), pady=(15, 15), sticky="E")

        # show the plot and row
        self.plotLabel = tk.Label(master, textvariable=self.plot, font=("Times New Roman", 14), bg="GhostWhite")
        self.plotLabel.grid(row=3, column=1, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        self.rowLabel = tk.Label(master, textvariable=self.row, font=("Times New Roman", 14), bg="GhostWhite")
        self.rowLabel.grid(row=4, column=1, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        
        #increment buttons
        self.plotup = tk.Button(master, text="Plot+", font=("Times New Roman", 14), command=lambda: self.incrementer(self.plot))
        self.plotdown = tk.Button(master, text="Plot-", font=("Times New Roman", 14), command=lambda: self.decrementer(self.plot))
        self.rowup = tk.Button(master, text="Row+", font=("Times New Roman", 14), command=lambda: self.incrementer(self.row))
        self.rowdown = tk.Button(master, text="Row-", font=("Times New Roman", 14), command=lambda: self.decrementer(self.row))
         
        #start and stop buttons
        self.start_btn = tk.Button(master, text="Start", font=("Times New Roman", 14), command= self.start_recording)
        self.stop_btn = tk.Button(master, text="Stop Recording", font=("Times New Roman", 14), command= self.stop_recording)

        # layout
        self.plotup.grid(row=3, column=2, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        self.plotdown.grid(row=3, column=3, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        self.rowup.grid(row=4, column=2, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        self.rowdown.grid(row=4, column=3, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
        self.start_btn.grid(row=5, column=2, padx=(15, 15), pady=(15, 15), sticky="EW")
        self.stop_btn.grid(row=6, column=2, padx=(15, 15), pady=(15, 15), sticky="EW")

        # Show recording Status
        self.status_lbl = tk.Label(master, text="Status", font=("Times New Roman", 14))
        self.recstat_lbl = tk.Label(master, textvariable=self.status, font=("Times New Roman", 14), bg="red")
        self.status_lbl.grid(row=7, column=0, padx=(15, 0), pady=(15, 15), sticky="NSE")
        self.recstat_lbl.grid(
                row=7, column=1, columnspan=3, padx=(15, 15), pady=(15, 15), sticky="NSW"
            )
        #show gps status
        self.RTKStat_lbl = tk.Label(master, text ="RTK fix?", font=("Times New Roman", 14), bg="gray69")
        self.RTKStat_lbl.grid(row=1, column=1, columnspan=1,  sticky="EW")
        self.RTKStatlabel = tk.Label(master, textvariable=self.RTKStat, font=("Times New Roman", 14), bg="GhostWhite")
        self.RTKStatlabel.grid(row=1, column=2, columnspan=1,  sticky="EW")
        
        #button Change Exposure
        self.exposureValueLabel = tk.Label(master, textvariable=self.exposure, font=("Times New Roman", 14), bg="GhostWhite")
        self.exposureValue = tk.Entry(master, font=("Times New Roman", 14), bg="GhostWhite")
        #self.expButton = tk.Button(master, text="Query Exposure", font=("Times New Roman", 14), command=self.exposure_query)
        #self.expChangeButton = tk.Button(master, text="Change Exposure", font=("Times New Roman", 14), command= lambda: exposure_change(exposureValue))
        #label describe whether exposure in Auto or set by user
        self.exposModeLabel = tk.Label(master, textvariable=self.expAuto, font=("Times New Roman", 14), bg="gray69")
        
        # camera feed
        self.camera_feeds = []
        for i in range(0, len(zed_list)):
            feed = tk.Label(master, text=self.name_list[i], font=("Times New Roman", 14))
            feed.grid(row=9, column=i, columnspan=1, padx=(15, 0), pady=(15, 15), sticky="EW")
            self.camera_feeds.append(feed)
            #self.draw_feed(i)

        #layout
        self.exposureValueLabel.grid(row=8, column=2, columnspan=1,  sticky="EW")
        self.exposureValue.grid(row=9, column=1, columnspan=1,  sticky="EW")
        #self.expButton.grid(row=8,column=4,columnspan=1,  sticky="EW")
        #self.expChangeButton.grid(row=8,column=3,columnspan=1,  sticky="EW")
        self.exposModeLabel.grid(row=8, column=1, columnspan=1,  sticky="EW")

    def incrementer(self,name):
        name.set(name.get()+1)

    def decrementer(self,name):
        name.set(name.get()-1)

    def rec_loop(self, id):
        """
        Video recording.
        id:  index zed_list[id]
        """
        # Recording path
        zedmetadata_fname = "_".join((self.metadata_fname, str(self.name_list[id])))
        zedrec_path = os.path.join(self.rec_path, zedmetadata_fname + ".svo")
        
        # Recording Parameters
        rec_params = sl.RecordingParameters(zedrec_path, sl.SVO_COMPRESSION_MODE.H265)

        # Enable Recording
        errrec = zed_list[id].enable_recording(rec_params)
        if errrec != sl.ERROR_CODE.SUCCESS:
            print(repr(errrec))
            zed_list[id].close()
            return
        
        print("\n[REC] ENABLed RECORDING")
        # Runtime Parameter
        runtime_params = sl.RuntimeParameters(enable_depth=True)
  
        while self.is_recording:
            framerr = zed_list[id].grab(runtime_params)  #grab frame  > to svo
            if framerr == sl.ERROR_CODE.SUCCESS:
                self.n_frames[id] += 1  #inc frame count
                zed_list[id].retrieve_image(left_list[id], sl.VIEW.LEFT)  #tell cam to retrieve info from grab()
                self.timestamp_list[id] = zed_list[id].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
                if self.timestamp_list[id] > self.last_ts_list[id]:
                    self.last_frames[id] = left_list[id].get_data()
                    self.last_ts_list[id] = self.timestamp_list[id]
                    photo = self.to_photo(self.last_frames[id])
                self.camera_feeds[id].photo_image = photo
                self.camera_feeds[id].configure(image = photo)
            elif framerr != sl.ERROR_CODE.SUCCESS:  #frame was not saved to SVO == Dropped
                self.dropped_count[id] +=1
            if self.is_recording == False:
                break
            #gnss_reader[index].grab()  #gps grab to svo
            self.RTKStat.set(1)   #update gui
            time.sleep(0.001) #1ms
        cv2.destroyAllWindows()

        # When the user asks to stop recording
        print("[REC] Recording Stopped")

        # Disable recording and print number of dropped frames
        zed_list[id].disable_recording()
        print("End recording and dropped",str(self.dropped_count[id]))
      
    # Start recording
    def start_recording(self):
        self.th = None
        self.thread_list = []
        
        if self.is_recording:
            print("Recordings Locked! Cannot start more. ")
            return 
            
        # Set recording flag to True
        self.is_recording = True

        # start clock
        self.start_time = time.perf_counter()

        # Update status label
        self.status.set("Recording Started")
        self.recstat_lbl.configure(bg="SpringGreen")
        # Filename
        self.metadata_fname = "_".join(["plot", str(self.plot.get()), "row", str(self.row.get())])
        
        # create recrding threads
        for i in range(0, len(zed_list)):
            if zed_list[i].is_opened():
                self.th = threading.Thread(target=self.rec_loop, args=(i,))
                #self.thread_list.append(self.th)
                self.th.start()

    def draw_feed(self, idx, feed_update_delay_ms=30):
        feed = self.camera_feeds[idx]
        if self.is_recording: 
            frame = self.last_frames[idx]
            if not frame is None and self.timestamp_list[idx] > self.last_ts_list[idx]:
                photo = self.to_photo(frame)
                feed.photo_image = photo
                feed.configure(image=photo)
        feed.after(feed_update_delay_ms, lambda: self.draw_feed(idx))

    def to_photo(self, frame):
        img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA))
        img = img.resize((480,300))    # 480x300 is 1980x1200 / 4
        return ImageTk.PhotoImage(image=img)

    def stop_recording(self):
        
        # If recording is already stopped
        if self.is_recording == False:
            print("Recording already stopped")
            return 

        # Reset the recording flag
        self.is_recording = False
        self.show_images = False
   
        # Update the recording status label
        self.status.set("Recording Stopped")
        self.recstat_lbl.configure(bg="red")
     
        #stop timer
        self.stop_time = time.perf_counter()
        self.timed = self.stop_time - self.start_time


        # Set csv metadata
        for i in range(0, len(zed_list)):
            self.metadata[i]["StartFrame"].append(0)
            self.metadata[i]["SN"] = self.name_list[i]
            self.metadata[i]["plot ID"].append(str(self.plot.get()))
            self.metadata[i]["row ID"].append(str(self.row.get()))
            self.metadata[i]["EndFrame"] = self.n_frames[i]
        self.metadata[0]["Time Elpsd"].append(self.timed)

        # Preparing and writing metadata output
        output_dict = {}
        for k in self.metadata[0].keys():
            output_dict[k] = self.metadata[0][k] , self.metadata[1][k] , self.metadata[2][k]

        # Write metadata to a csv file
        with open(
            os.path.join(self.rec_path, self.metadata_fname) + ".csv", "w", newline=""
        ) as output_file:
            writer = csv.writer(output_file)
            writer.writerow(list(output_dict.keys()))
            writer.writerows(zip(*output_dict.values()))
        print("Data has been written")
        
        # Reset metadata for next recording
        self.metadata = [
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
            {
                "SN": [],
                "plot ID": [],
                "row ID": [],
                "# Frames": [],
                "StartFrame": [],
                "EndFrame": [],
                "Time Elpsd": [],
            },
        ]
        self.metadata_fname = ""

def on_close(evt=None):
    """
            This function handles the the close button.
    """
              
    if messagebox.askyesno("Close", message="Are you sure you wish to exit?"):  
        print("\n\nReceived Exit Signal!\n")
        # Stop any threads that are recording
        time.sleep(0.5)  
        for zed in zed_list:                       
            zed.close() 
            print("Zed is open?", zed.is_opened())                    # Close the camera
        root.destroy()
        sys.exit(0)                         # Exit


def record(export_path="/home/erin/Pyzedgui/Testing"):
    global root
    root = tk.Tk()
    app = ZEDRecording(root, export_path)
    root.bind('<Escape>', on_close)
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    record()
