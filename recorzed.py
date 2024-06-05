
import pyzed.sl as sl 
import threading
import time

'''class for zed.  call per zed. this opens cameras, enables recording, and cleans up '''
class RecordZed():
    def __init__(self) -> None:
        self.zed = sl.Camera()
        self.left_frame = sl.Mat()
        self.continue_to_grab = True
        self.new_data = False
        self.frame_getter = None
        self.grab_zed_data = None
        self.is_initialized_mtx = threading.Lock()
        self.is_initialized = False
        
    def initialize(self):
        # Set Init Parameters
        print("\n[INFO] SETTING INIT PARAMETERS")
        init_params = sl.InitParameters(
            camera_resolution=sl.RESOLUTION.AUTO,
            camera_fps=30,
            depth_mode=sl.DEPTH_MODE.PERFORMANCE,
            sdk_verbose =0
        )

        # Obtain available devices' serial numbers
        #available_devices = sl.Camera.get_device_list()
        #index = 0
        #for dev in available_devices:
        
        #INIT CAMERAS
        init_params.set_from_serial_number(self.zed.serial_number)
        #open
        openerr = self.zed.open(init_params)
        if openerr != sl.ERROR_CODE.SUCCESS:
            print(repr(openerr))
            self.zed.close()
            return -1, "Erro"
        with self.is_initialized_mtx:
            self.is_initialized = True
        return 0, self.zed.serial_number

    def startRecording(self):
        self.grab_zed_data = threading.Thread(target=self.grabZEDData)
        self.grab_zed_data.start()
        
    def grab(self):
        if self.new_data:
            self.new_data = False
            return sl.ERROR_CODE.SUCCESS, self.left_frame
        return sl.ERROR_CODE.FAILURE, None

    def grabZEDData(self):
        while self.continue_to_grab:
            with self.is_initialized_mtx:
                if self.is_initialized:
                    break
            time.sleep(0.001)
        while self.continue_to_grab:
            self.left_frame = self.getNextFrame()
            if self.left_frame is not None:
                self.new_data = True

    def stop_thread(self):
        self.continue_to_grab = False   

    def getNextFrame(self) -> sl.Mat:
        if self.new_data:
            err = self.zed.grab(runiime)
            if err == success:
                self.zed.retrieveiomage()self.left_frame
        return self.left_frame
    
    def exposure(self):
        """
         exposure values & frame
        run query before running change
        """
        runtime = sl.RuntimeParameters()
        
        err, exp = self.zed.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME)
        if err == sl.ERROR_CODE.SUCCESS:
            errg = self.zed.grab(runtime)  #grab a frame at this epsoure value
            if errg == sl.ERROR_CODE.SUCCESS: 
                self.zed.retrieve_image(self.left_frame, sl.VIEW.LEFT) #save left view to left_frame   
        else:
            exp= -1
            self.left_frame = None
        return exp, self.left_frame
               
    def exposure_control(self,entry):
        """
        control exposure for different lighting conditions. 
        Textbox to control current exposure value
        This will not change the exposure value for the top camera
        """
        if entry >= 100 and entry < 6599: 
                err = self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME, entry)
                if err == sl.ERROR_CODE.SUCCESS:
                    err, exset = self.zed.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME)
                    if err == sl.ERROR_CODE.SUCCESS:
                        status = "Manual exp"
                        #print("Current exposure for Zed ",str(self.name_list[i]), ":", tryvalue)
                else:
                    print("Unable to set exposure time", entry) 
                    status = "Error"
        return status, exset
    def exposure_auto(self):
        """
        reset exposure to AUTO
        """
        err = self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME, -1)
        if err == sl.ERROR_CODE.SUCCESS:
            status = "Auto exp" 
            err, expvalue = self.zed.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME)
            if err == sl.ERROR_CODE.SUCCESS:
                pass
            else:
                print("error for Zed ",str(self.zed.serial_number), ":", err)
        else:
            print("Unable to REset exposure") 
            status = " ".join("Error.",err)  
            expvalue = 0
        return status, expvalue
    