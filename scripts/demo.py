#!/usr/bin/env python
# coding: utf-8
'''
Created on 2017-10-25

@author: zhangruijun
'''

from email.header import Header
from operator import is_not
import ctypes
from turtle import stamp
# from pickletools import uint8
from ImageConvert import *
from MVSDK import *
import struct
import time
import datetime
import rospy
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header as hd
import cv2
from cv_bridge import CvBridge,CvBridgeError
import gc

# code dict for para SerialNumber
# "7J"----------"7J0AC6BPAK00005"
# others----------"6L0771APAK00041"
glob_camserial = "7J0AC6BPAK00005" if rospy.get_param("SerialNumber") == "7J" else "6L0771APAK00041"
 
g_cameraStatusUserInfo = b"statusInfo"

class iRAYPLE():
    def __init__(self, camera):
        rospy.init_node("publisher")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.camera = camera
        # self.id = str(index)
        self.key = camera.getKey(camera)
        self.serialnum = str(camera.getSerialNumber(camera))
        print(self.serialnum)
        self.connectCallBackFuncEx = connectCallBackEx(self.deviceLinkNotify)
        self.frameCallbackFuncEx = callbackFuncEx(self.onGetFrameEx)
        self.acq_count = 0
        self.Brightness = rospy.get_param("Brightness")#100
        self.Exposuretime = rospy.get_param("Exposuretime")#20000
        self.Framrate = rospy.get_param("Framerate")#22
        self.shoottime = rospy.get_param("Shoottime")#30
        self.NUM_IMAGES = self.shoottime*self.Framrate
        self.AutoExposure = rospy.get_param("AutoExposure")#True
        self.images = []
        self.buff = list()
        self.timestamps = []
        self.userInfo = b"test"

        self.bridge = CvBridge()
        self.pub = rospy.Publisher("camera"+rospy.get_param("SerialNumber")+"_Image", Image, queue_size=10000)
        nRet = self.initCamera()
        if( nRet == -1 ):
            print("initCamera failed")
            # 释放相关资源
            self.streamSource.contents.release(self.streamSource)   
        elif(nRet == -2):
            print("initfailed by open")

    def setAcquisitionmode(self,mode = b"Continuous"):
        AcqModeEnumNode = pointer(GENICAM_EnumNode())
        AcqModeEnumNodeInfo = GENICAM_EnumNodeInfo() 
        AcqModeEnumNodeInfo.pCamera = pointer(self.camera)
        AcqModeEnumNodeInfo.attrName = b"AcquisitionMode"
        nRet = GENICAM_createEnumNode(byref(AcqModeEnumNodeInfo), byref(AcqModeEnumNode))
        if ( nRet != 0 ):
            print("create Acquisitionmode Node fail!")
            # 释放相关资源
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        nRet = AcqModeEnumNode.contents.setValueBySymbol(AcqModeEnumNode, mode)
        if ( nRet != 0 ):
            print("set Acquisitionmode value [Continuous] fail!")
            # 释放相关资源
            AcqModeEnumNode.contents.release(AcqModeEnumNode)
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        # 需要释放Node资源    
        AcqModeEnumNode.contents.release(AcqModeEnumNode) 
        return 0 

    def setFramerate(self,fr):
        # Set Frame Rate
        FRNode = pointer(GENICAM_DoubleNode())
        FRNodeInfo = GENICAM_DoubleNodeInfo() 
        FRNodeInfo.pCamera = pointer(self.camera)
        FRNodeInfo.attrName = b"AcquisitionFrameRate"
        nRet = GENICAM_createDoubleNode(byref(FRNodeInfo), byref(FRNode))
        if ( nRet != 0 ):
            print("create FR Node fail!")
            return -1
        
        # 设置曝光时间
        nRet = FRNode.contents.setValue(FRNode, c_double(fr))  
        if ( nRet != 0 ):
            print("set FR value [%f]us fail!"  % (fr))
            # 释放相关资源
            FRNode.contents.release(FRNode)
            return -1
        else:
            print("set FR value [%f]us success." % (fr))
                
        # 释放节点资源     
        FRNode.contents.release(FRNode)    

        # Set Frame Rate Enable
        FRENode = pointer(GENICAM_BoolNode())
        FRENodeInfo = GENICAM_BoolNodeInfo() 
        FRENodeInfo.pCamera = pointer(self.camera)
        FRENodeInfo.attrName = b"AcquisitionFrameRateEnable"
        nRet = GENICAM_createBoolNode(byref(FRENodeInfo), byref(FRENode))
        if ( nRet != 0 ):
            print("create FRE Node fail!")
            return -1
        
        # 设置曝光时间
        nRet = FRENode.contents.setValue(FRENode, True)  
        if ( nRet != 0 ):
            print("set FRE value true fail!")
            # 释放相关资源
            FRENode.contents.release(FRENode)
            return -1
        else:
            print("set FRE value true success.")
                
        # 释放节点资源     
        FRENode.contents.release(FRENode)    
        return 0

    def cam_deinit(self):
        # 关闭相机
        nRet = self.closeCamera()
        if ( nRet != 0 ):
            print("closeCamera fail")
            # 释放相关资源
            self.streamSource.contents.release(self.streamSource)   
            return -1
        
        # 释放相关资源
        self.streamSource.contents.release(self.streamSource)    

    # 取流回调函数Ex
    def onGetFrameEx(self, frame, userInfo):
        nRet = frame.contents.valid(frame)
        if ( nRet != 0):
            print("frame is invalid!")
            # 释放驱动图像缓存资源
            frame.contents.release(frame)
            return         
        
        self.acq_count+=1
        #此处客户应用程序应将图像拷贝出使用
        # 给转码所需的参数赋值
        imageParams = IMGCNV_SOpenParam()
        imageParams.dataSize    = frame.contents.getImageSize(frame)
        imageParams.height      = frame.contents.getImageHeight(frame)
        imageParams.width       = frame.contents.getImageWidth(frame)
        imageParams.paddingX    = frame.contents.getImagePaddingX(frame)
        imageParams.paddingY    = frame.contents.getImagePaddingY(frame)
        imageParams.pixelForamt = frame.contents.getImagePixelFormat(frame)
        ts = frame.contents.getImageTimeStamp(frame)

        # 将裸数据图像拷出
        imageBuff = frame.contents.getImage(frame)
        userBuff = c_buffer(b'\0', imageParams.dataSize)
        memmove(userBuff, c_char_p(imageBuff), imageParams.dataSize)

        # 释放驱动图像缓存资源
        frame.contents.release(frame)

        # 如果图像格式是 Mono8 直接使用
        if imageParams.pixelForamt == EPixelType.gvspPixelMono8:
            grayByteArray = bytearray(userBuff)
            cvImage = numpy.array(grayByteArray).reshape(imageParams.height, imageParams.width)
        else:
            # 转码 => BGR24
            rgbSize = c_int()
            rgbBuff = c_buffer(b'\0', imageParams.height * imageParams.width * 3)

            nRet = IMGCNV_ConvertToBGR24(cast(userBuff, c_void_p), \
                                        byref(imageParams), \
                                        cast(rgbBuff, c_void_p), \
                                        byref(rgbSize))

            colorByteArray = bytearray(rgbBuff)
            cvImage = numpy.array(colorByteArray).reshape(imageParams.height, imageParams.width, 3)
        msg_temp = self.bridge.cv2_to_imgmsg(cvImage,'bgr8')
        msg_temp.header.frame_id = str(ts)
        self.pub.publish(msg_temp)
    # --- end if ---

        # cv2.imshow('myWindow', cvImage)
        gc.collect()


    # 相机连接状态回调函数
    def deviceLinkNotify(self, connectArg, linkInfo):
        if ( EVType.offLine == connectArg.contents.m_event ):
            print("camera has off line, userInfo [%s]" %(c_char_p(linkInfo).value))
        elif ( EVType.onLine == connectArg.contents.m_event ):
            print("camera has on line, userInfo [%s]" %(c_char_p(linkInfo).value))
        
    # 注册相机连接状态回调
    def subscribeCameraStatus(self):
        # 注册上下线通知
        eventSubscribe = pointer(GENICAM_EventSubscribe())
        eventSubscribeInfo = GENICAM_EventSubscribeInfo()
        eventSubscribeInfo.pCamera = pointer(self.camera)
        nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
        if ( nRet != 0):
            print("create eventSubscribe fail!")
            return -1
        
        nRet = eventSubscribe.contents.subscribeConnectArgsEx(eventSubscribe, self.connectCallBackFuncEx, g_cameraStatusUserInfo)
        if ( nRet != 0 ):
            print("subscribeConnectArgsEx fail!")
            # 释放相关资源
            eventSubscribe.contents.release(eventSubscribe)
            return -1  
        
        # 不再使用时，需释放相关资源
        eventSubscribe.contents.release(eventSubscribe) 
        return 0

    # 反注册相机连接状态回调
    def unsubscribeCameraStatus(self):
        # 反注册上下线通知
        eventSubscribe = pointer(GENICAM_EventSubscribe())
        eventSubscribeInfo = GENICAM_EventSubscribeInfo()
        eventSubscribeInfo.pCamera = pointer(self.camera)
        nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
        if ( nRet != 0):
            print("create eventSubscribe fail!")
            return -1
            
        nRet = eventSubscribe.contents.unsubscribeConnectArgsEx(eventSubscribe, self.connectCallBackFuncEx, g_cameraStatusUserInfo)
        if ( nRet != 0 ):
            print("unsubscribeConnectArgsEx fail!")
            # 释放相关资源
            eventSubscribe.contents.release(eventSubscribe)
            return -1
        
        # 不再使用时，需释放相关资源
        eventSubscribe.contents.release(eventSubscribe)
        return 0   

    # 设置软触发
    def setSoftTriggerConf(self):
        # 创建control节点
        acqCtrlInfo = GENICAM_AcquisitionControlInfo()
        acqCtrlInfo.pCamera = pointer(self.camera)
        acqCtrl = pointer(GENICAM_AcquisitionControl())
        nRet = GENICAM_createAcquisitionControl(pointer(acqCtrlInfo), byref(acqCtrl))
        if ( nRet != 0 ):
            print("create AcquisitionControl fail!")
            return -1
        
        # 设置触发源为软触发
        trigSourceEnumNode = acqCtrl.contents.triggerSource(acqCtrl)
        nRet = trigSourceEnumNode.setValueBySymbol(byref(trigSourceEnumNode), b"Software")
        if ( nRet != 0 ):
            print("set TriggerSource value [Software] fail!")
            # 释放相关资源
            trigSourceEnumNode.release(byref(trigSourceEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return -1
        
        # 需要释放Node资源
        trigSourceEnumNode.release(byref(trigSourceEnumNode))
        
        # 设置触发方式
        trigSelectorEnumNode = acqCtrl.contents.triggerSelector(acqCtrl)
        nRet = trigSelectorEnumNode.setValueBySymbol(byref(trigSelectorEnumNode), b"FrameStart")
        if ( nRet != 0 ):
            print("set TriggerSelector value [FrameStart] fail!")
            # 释放相关资源
            trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
            acqCtrl.contents.release(acqCtrl) 
            return -1
        
        # 需要释放Node资源    
        trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
        
        # 打开触发模式
        trigModeEnumNode = acqCtrl.contents.triggerMode(acqCtrl)
        nRet = trigModeEnumNode.setValueBySymbol(byref(trigModeEnumNode), b"On")
        if ( nRet != 0 ):
            print("set TriggerMode value [On] fail!")
            # 释放相关资源
            trigModeEnumNode.release(byref(trigModeEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return -1
        
        # 需要释放相关资源    
        trigModeEnumNode.release(byref(trigModeEnumNode))
        acqCtrl.contents.release(acqCtrl)
        
        return 0     

    # 设置外触发
    def setLineTriggerConf(self):
        # 创建control节点
        acqCtrlInfo = GENICAM_AcquisitionControlInfo()
        acqCtrlInfo.pCamera = pointer(self.camera)
        acqCtrl = pointer(GENICAM_AcquisitionControl())
        nRet = GENICAM_createAcquisitionControl(pointer(acqCtrlInfo), byref(acqCtrl))
        if ( nRet != 0 ):
            print("create AcquisitionControl fail!")
            return -1
        
        # 设置触发源为软触发
        trigSourceEnumNode = acqCtrl.contents.triggerSource(acqCtrl)
        nRet = trigSourceEnumNode.setValueBySymbol(byref(trigSourceEnumNode), b"Line1")
        if ( nRet != 0 ):
            print("set TriggerSource value [Line1] fail!")
            # 释放相关资源
            trigSourceEnumNode.release(byref(trigSourceEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return -1
        
        # 需要释放Node资源
        trigSourceEnumNode.release(byref(trigSourceEnumNode))
        
        # 设置触发方式
        trigSelectorEnumNode = acqCtrl.contents.triggerSelector(acqCtrl)
        nRet = trigSelectorEnumNode.setValueBySymbol(byref(trigSelectorEnumNode), b"FrameStart")
        if ( nRet != 0 ):
            print("set TriggerSelector value [FrameStart] fail!")
            # 释放相关资源
            trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
            acqCtrl.contents.release(acqCtrl) 
            return -1
        
        # 需要释放Node资源    
        trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
        
        # 打开触发模式
        trigModeEnumNode = acqCtrl.contents.triggerMode(acqCtrl)
        nRet = trigModeEnumNode.setValueBySymbol(byref(trigModeEnumNode), b"On")
        if ( nRet != 0 ):
            print("set TriggerMode value [On] fail!")
            # 释放相关资源
            trigModeEnumNode.release(byref(trigModeEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return -1
        
        # 需要释放Node资源    
        trigModeEnumNode.release(byref(trigModeEnumNode))
        
        # 设置触发沿
        trigActivationEnumNode = acqCtrl.contents.triggerActivation(acqCtrl)
        nRet = trigActivationEnumNode.setValueBySymbol(byref(trigActivationEnumNode), b"RisingEdge")
        if ( nRet != 0 ):
            print("set TriggerActivation value [RisingEdge] fail!")
            # 释放相关资源
            trigActivationEnumNode.release(byref(trigActivationEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return -1
        
        # 需要释放Node资源    
        trigActivationEnumNode.release(byref(trigActivationEnumNode))
        acqCtrl.contents.release(acqCtrl)
        return 0 

    # 打开相机
    def openCamera(self):
        # 连接相机
        nRet = self.camera.connect(self.camera, c_int(GENICAM_ECameraAccessPermission.accessPermissionControl))
        if ( nRet != 0 ):
            print("camera connect fail!")
            return -1
        else:
            print("camera connect success.")
    
        # 注册相机连接状态回调
        nRet = self.subscribeCameraStatus()
        if ( nRet != 0 ):
            print("subscribeCameraStatus fail!")
            return -1

        return 0

    # 关闭相机
    def closeCamera(self):
        # 反注册相机连接状态回调
        nRet = self.unsubscribeCameraStatus()
        if ( nRet != 0 ):
            print("unsubscribeCameraStatus fail!")
            return -1
    
        # 断开相机
        nRet = self.camera.disConnect(byref(self.camera))
        if ( nRet != 0 ):
            print("disConnect camera fail!")
            return -1
        
        return 0    

    # set exp auto
    def setExposureAuto(self):
        ExpAutoEnumNode = pointer(GENICAM_EnumNode())
        ExpAutoEnumNodeInfo = GENICAM_EnumNodeInfo() 
        ExpAutoEnumNodeInfo.pCamera = pointer(self.camera)
        ExpAutoEnumNodeInfo.attrName = b"ExposureAuto"
        nRet = GENICAM_createEnumNode(byref(ExpAutoEnumNodeInfo), byref(ExpAutoEnumNode))
        if ( nRet != 0 ):
            print("create AutoeExposure Node fail!")
            # 释放相关资源
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        nRet = ExpAutoEnumNode.contents.setValueBySymbol(ExpAutoEnumNode, b"Continuous")
        if ( nRet != 0 ):
            print("set AutoExposure value [Continuous] fail!")
            # 释放相关资源
            ExpAutoEnumNode.contents.release(ExpAutoEnumNode)
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        # 需要释放Node资源    
        ExpAutoEnumNode.contents.release(ExpAutoEnumNode) 
        return 0 

    # 设置曝光
    def setExposureTime(self, dVal):
        # 通用属性设置:设置曝光 --根据属性类型，直接构造属性节点。如曝光是 double类型，构造doubleNode节点
        exposureTimeNode = pointer(GENICAM_DoubleNode())
        exposureTimeNodeInfo = GENICAM_DoubleNodeInfo() 
        exposureTimeNodeInfo.pCamera = pointer(self.camera)
        exposureTimeNodeInfo.attrName = b"ExposureTime"
        nRet = GENICAM_createDoubleNode(byref(exposureTimeNodeInfo), byref(exposureTimeNode))
        if ( nRet != 0 ):
            print("create ExposureTime Node fail!")
            return -1
        
        # 设置曝光时间
        nRet = exposureTimeNode.contents.setValue(exposureTimeNode, c_double(dVal))  
        if ( nRet != 0 ):
            print("set ExposureTime value [%f]us fail!"  % (dVal))
            # 释放相关资源
            exposureTimeNode.contents.release(exposureTimeNode)
            return -1
        else:
            print("set ExposureTime value [%f]us success." % (dVal))
                
        # 释放节点资源     
        exposureTimeNode.contents.release(exposureTimeNode)    
        return 0

    # set Brightness
    def setBrightness(self, iVal):
        
        BrightnessNode = pointer(GENICAM_IntNode())
        BrightnessNodeInfo = GENICAM_IntNodeInfo() 
        BrightnessNodeInfo.pCamera = pointer(self.camera)
        BrightnessNodeInfo.attrName = b"Brightness"
        nRet = GENICAM_createIntNode(byref(BrightnessNodeInfo), byref(BrightnessNode))
        if ( nRet != 0 ):
            print("create Brightness Node fail!")
            return -1
        
        # 设置Brightness
        nRet = BrightnessNode.contents.setValue(BrightnessNode, (iVal))  
        if ( nRet != 0 ):
            print("set Brightness value [%d]us fail!"  % (iVal))
            # 释放相关资源
            BrightnessNode.contents.release(BrightnessNode)
            return -1
        else:
            print("set Brightness value [%d]us success." % (iVal))
                
        # 释放节点资源     
        BrightnessNode.contents.release(BrightnessNode)    
        return 0

    def setTriggerModeoff(self):
        # 通用属性设置:设置触发模式为off --根据属性类型，直接构造属性节点。如触发模式是 enumNode，构造enumNode节点
        # 自由拉流：TriggerMode 需为 off
        trigModeEnumNode = pointer(GENICAM_EnumNode())
        trigModeEnumNodeInfo = GENICAM_EnumNodeInfo() 
        trigModeEnumNodeInfo.pCamera = pointer(self.camera)
        trigModeEnumNodeInfo.attrName = b"TriggerMode"
        nRet = GENICAM_createEnumNode(byref(trigModeEnumNodeInfo), byref(trigModeEnumNode))
        if ( nRet != 0 ):
            print("create TriggerMode Node fail!")
            # 释放相关资源
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        nRet = trigModeEnumNode.contents.setValueBySymbol(trigModeEnumNode, b"Off")
        if ( nRet != 0 ):
            print("set TriggerMode value [Off] fail!")
            # 释放相关资源
            trigModeEnumNode.contents.release(trigModeEnumNode)
            self.streamSource.contents.release(self.streamSource) 
            return -1
        
        # 需要释放Node资源    
        trigModeEnumNode.contents.release(trigModeEnumNode) 
        return 0
        

    def grabOne(self):
        # 创建流对象
        streamSourceInfo = GENICAM_StreamSourceInfo()
        streamSourceInfo.channelId = 0
        streamSourceInfo.pCamera = pointer(self.camera)
        
        streamSource = pointer(GENICAM_StreamSource())
        nRet = GENICAM_createStreamSource(pointer(streamSourceInfo), byref(streamSource))
        if ( nRet != 0 ):
            print("create StreamSource fail!")     
            return -1
        
        # 创建control节点
        acqCtrlInfo = GENICAM_AcquisitionControlInfo()
        acqCtrlInfo.pCamera = pointer(self.camera)
        acqCtrl = pointer(GENICAM_AcquisitionControl())
        nRet = GENICAM_createAcquisitionControl(pointer(acqCtrlInfo), byref(acqCtrl))
        if ( nRet != 0 ):
            print("create AcquisitionControl fail!")
            # 释放相关资源
            streamSource.contents.release(streamSource)  
            return -1
            
        # 执行一次软触发
        trigSoftwareCmdNode = acqCtrl.contents.triggerSoftware(acqCtrl)
        nRet = trigSoftwareCmdNode.execute(byref(trigSoftwareCmdNode))
        if( nRet != 0 ):
            print("Execute triggerSoftware fail!")
            # 释放相关资源
            trigSoftwareCmdNode.release(byref(trigSoftwareCmdNode))
            acqCtrl.contents.release(acqCtrl)
            streamSource.contents.release(streamSource)   
            return -1   

        # 释放相关资源
        trigSoftwareCmdNode.release(byref(trigSoftwareCmdNode))
        acqCtrl.contents.release(acqCtrl)
        streamSource.contents.release(streamSource) 
        
        return 0  

    # 设置感兴趣区域  --- 感兴趣区域的宽高 和 xy方向的偏移量  入参值应符合对应相机的递增规则
    def setROI(self, OffsetX, OffsetY, nWidth, nHeight):
        #获取原始的宽度
        widthMaxNode = pointer(GENICAM_IntNode())
        widthMaxNodeInfo = GENICAM_IntNodeInfo() 
        widthMaxNodeInfo.pCamera = pointer(self.camera)
        widthMaxNodeInfo.attrName = b"WidthMax"
        nRet = GENICAM_createIntNode(byref(widthMaxNodeInfo), byref(widthMaxNode))
        if ( nRet != 0 ):
            print("create WidthMax Node fail!")
            return -1
        
        oriWidth = c_longlong()
        nRet = widthMaxNode.contents.getValue(widthMaxNode, byref(oriWidth))
        if ( nRet != 0 ):
            print("widthMaxNode getValue fail!")
            # 释放相关资源
            widthMaxNode.contents.release(widthMaxNode)
            return -1  
        
        # 释放相关资源
        widthMaxNode.contents.release(widthMaxNode)
        
        # 获取原始的高度
        heightMaxNode = pointer(GENICAM_IntNode())
        heightMaxNodeInfo = GENICAM_IntNodeInfo() 
        heightMaxNodeInfo.pCamera = pointer(self.camera)
        heightMaxNodeInfo.attrName = b"HeightMax"
        nRet = GENICAM_createIntNode(byref(heightMaxNodeInfo), byref(heightMaxNode))
        if ( nRet != 0 ):
            print("create HeightMax Node fail!")
            return -1
        
        oriHeight = c_longlong()
        nRet = heightMaxNode.contents.getValue(heightMaxNode, byref(oriHeight))
        if ( nRet != 0 ):
            print("heightMaxNode getValue fail!")
            # 释放相关资源
            heightMaxNode.contents.release(heightMaxNode)
            return -1
        
        # 释放相关资源
        heightMaxNode.contents.release(heightMaxNode)
            
        # 检验参数
        if ( ( oriWidth.value < (OffsetX + nWidth)) or ( oriHeight.value < (OffsetY + nHeight)) ):
            print("please check input param!")
            return -1
        
        # 设置宽度
        widthNode = pointer(GENICAM_IntNode())
        widthNodeInfo = GENICAM_IntNodeInfo() 
        widthNodeInfo.pCamera = pointer(self.camera)
        widthNodeInfo.attrName = b"Width"
        nRet = GENICAM_createIntNode(byref(widthNodeInfo), byref(widthNode))
        if ( nRet != 0 ):
            print("create Width Node fail!") 
            return -1
        
        nRet = widthNode.contents.setValue(widthNode, c_longlong(nWidth))
        if ( nRet != 0 ):
            print("widthNode setValue [%d] fail!" % (nWidth))
            # 释放相关资源
            widthNode.contents.release(widthNode)
            return -1  
        
        # 释放相关资源
        widthNode.contents.release(widthNode)
        
        # 设置高度
        heightNode = pointer(GENICAM_IntNode())
        heightNodeInfo = GENICAM_IntNodeInfo() 
        heightNodeInfo.pCamera = pointer(self.camera)
        heightNodeInfo.attrName = b"Height"
        nRet = GENICAM_createIntNode(byref(heightNodeInfo), byref(heightNode))
        if ( nRet != 0 ):
            print("create Height Node fail!")
            return -1
        
        nRet = heightNode.contents.setValue(heightNode, c_longlong(nHeight))
        if ( nRet != 0 ):
            print("heightNode setValue [%d] fail!" % (nHeight))
            # 释放相关资源
            heightNode.contents.release(heightNode)
            return -1    
        
        # 释放相关资源
        heightNode.contents.release(heightNode)    
        
        # 设置OffsetX
        OffsetXNode = pointer(GENICAM_IntNode())
        OffsetXNodeInfo = GENICAM_IntNodeInfo() 
        OffsetXNodeInfo.pCamera = pointer(self.camera)
        OffsetXNodeInfo.attrName = b"OffsetX"
        nRet = GENICAM_createIntNode(byref(OffsetXNodeInfo), byref(OffsetXNode))
        if ( nRet != 0 ):
            print("create OffsetX Node fail!")
            return -1
        
        nRet = OffsetXNode.contents.setValue(OffsetXNode, c_longlong(OffsetX))
        if ( nRet != 0 ):
            print("OffsetX setValue [%d] fail!" % (OffsetX))
            # 释放相关资源
            OffsetXNode.contents.release(OffsetXNode)
            return -1    
        
        # 释放相关资源
        OffsetXNode.contents.release(OffsetXNode)  
        
        # 设置OffsetY
        OffsetYNode = pointer(GENICAM_IntNode())
        OffsetYNodeInfo = GENICAM_IntNodeInfo() 
        OffsetYNodeInfo.pCamera = pointer(self.camera)
        OffsetYNodeInfo.attrName = b"OffsetY"
        nRet = GENICAM_createIntNode(byref(OffsetYNodeInfo), byref(OffsetYNode))
        if ( nRet != 0 ):
            print("create OffsetY Node fail!")
            return -1
        
        nRet = OffsetYNode.contents.setValue(OffsetYNode, c_longlong(OffsetY))
        if ( nRet != 0 ):
            print("OffsetY setValue [%d] fail!" % (OffsetY))
            # 释放相关资源
            OffsetYNode.contents.release(OffsetYNode)
            return -1    
        
        # 释放相关资源
        OffsetYNode.contents.release(OffsetYNode)   
        return 0


    def initCamera(self):
        # 打开相机
        nRet = self.openCamera()
        if ( nRet != 0 ):
            print("openCamera fail.")
            return -2
            
        # 创建流对象
        streamSourceInfo = GENICAM_StreamSourceInfo()
        streamSourceInfo.channelId = 0
        streamSourceInfo.pCamera = pointer(self.camera)
        
        self.streamSource = pointer(GENICAM_StreamSource())
        nRet = GENICAM_createStreamSource(pointer(streamSourceInfo), byref(self.streamSource))
        if ( nRet != 0 ):
            print("create StreamSource fail!")
            return -1
        
        if self.AutoExposure :
            nRet = self.setExposureAuto()
            if ( nRet != 0 ):
                print("setExposureAuto fail.")
                return -1
        else:
            # 设置曝光
            nRet = self.setExposureTime(self.Exposuretime)
            if ( nRet != 0 ):
                print("set ExposureTime fail")
                return -1

        nRet = self.setAcquisitionmode()
        if ( nRet != 0 ):
            print("setAcqMode fail.")
            return -1

        nRet = self.setFramerate(self.Framrate)
        if ( nRet != 0 ):
            print("setFR fail.")
            return -1

        #Set Trigger
        nRet = self.setTriggerModeoff()
        if ( nRet != 0 ):
            print("setTriggerModeoff fail.")
            return -1

        #Set Brightness
        nRet = self.setBrightness(100)
        if ( nRet != 0 ):
            print("set ExposureTime fail")
            return -1

        # # 设置软触发
        # nRet = self.setSoftTriggerConf()
        # if ( nRet != 0 ):
        #     print("set SoftTriggerConf fail!")
        #     return -1
        # else:
        #     print("set SoftTriggerConf success!")
        return 0
                        
    # start acq: register_callback + start_streaming
    def startAcquisition(self):
        nRet = self.streamSource.contents.attachGrabbingEx(self.streamSource, self.frameCallbackFuncEx, self.userInfo)    
        if ( nRet != 0 ):
            print("attachGrabbingEx fail!")
            return -1
            
        # 开始拉流
        nRet = self.streamSource.contents.startGrabbing(self.streamSource, c_ulonglong(0), \
                                                c_int(GENICAM_EGrabStrategy.grabStrartegySequential))
        if( nRet != 0):
            print("startGrabbing fail!")
            return -1
        return 0

    # end acq
    def endAcquisition(self):
        # 反注册回调函数
        nRet = self.streamSource.contents.detachGrabbingEx(self.streamSource, self.frameCallbackFuncEx, self.userInfo) 
        if ( nRet != 0 ):
            print("detachGrabbingEx fail!")
            return -1
            
        # 停止拉流
        nRet = self.streamSource.contents.stopGrabbing(self.streamSource)
        if ( nRet != 0 ):
            print("stopGrabbing fail!")  
            return -1
        return 0

    #main function of camera
    def runfreestream(self):
        nRet = self.startAcquisition()
        if ( nRet != 0 ):
            print("startAcquisition fail!")  
            self.streamSource.contents.release(self.streamSource)   
            return -1
        if self.shoottime==0:
            while(not rospy.is_shutdown()):
                pass
        else:
            while(self.acq_count<self.NUM_IMAGES):
                # print(self.acq_count)
                pass
        print("---------------total number is "+str(self.acq_count)+"---------------")
        print("img buffer count: ",str(device1.images.count))
        nRet = self.endAcquisition()
        if ( nRet != 0 ):
            print("endAcquisition fail!")  
            self.streamSource.contents.release(self.streamSource)   
            return -1
        return 0
# 枚举相机
def enumCameras():
    # 获取系统单例
    system = pointer(GENICAM_System())
    nRet = GENICAM_getSystemInstance(byref(system))
    if ( nRet != 0 ):
        print("getSystemInstance fail!")
        return None, None

    # 发现相机 
    cameraList = pointer(GENICAM_Camera()) 
    cameraCnt = c_uint()
    nRet = system.contents.discovery(system, byref(cameraList), byref(cameraCnt), c_int(GENICAM_EProtocolType.typeAll));
    if ( nRet != 0 ):
        print("discovery fail!")
        return None, None
    elif cameraCnt.value < 1:
        print("discovery no camera!")
        return None, None
    else:
        print("cameraCnt: " + str(cameraCnt.value))
        return cameraCnt.value, cameraList

if __name__=="__main__": 
    cameraCnt, cameraList = enumCameras()
    if cameraCnt is not None:
        # 显示相机信息
        init = 0 #flag of finding camera
        for index in range(0, cameraCnt):
            camera = cameraList[index]
            if str(camera.getSerialNumber(camera))== glob_camserial:
                device1 = iRAYPLE(camera)
                init = 1
        if init == 1:
            
            print("Start Acquisition")
            
            nRet = device1.runfreestream()
            if nRet != 0:
                print("free streamfailed")

            print("Acquisition Finished")

            # end
            device1.cam_deinit()
        else:
            print("No Camera with Number "+str(glob_camserial))
        
    else:
        print("no camera detected")
    print("--------- end ---------")
    # 3s exit
    time.sleep(3) 	


    
