
import cv2
import numpy as np
from scipy import ndimage
import time
import math
import sys
import os
import matplotlib.animation as animation
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import csv
import random
import concurrent.futures
from multiprocessing import Process,Value,Manager
import ctypes
import serial
import pandas as pd
import random

class SC800IM700:
  def __init__(self, ser_port, I2C_address):

    #シリアル通信の開始
    self.ser = serial.Serial(ser_port, baudrate=9600,bytesize=8, parity='N', stopbits=1, timeout=0.01)

    #I2Cデバイスのアドレス定義
    self.read_address =  ((I2C_address << 1) | 0x01).to_bytes(1,'big') #0x55
    self.write_address = ((I2C_address << 1) & 0xfe).to_bytes(1,'big')  #0x54
    self.address = I2C_address.to_bytes(1, 'big')

    #各種コマンド
    self.S = b'\x53' #I2C通信開始
    self.P = b'\x50' #I2C通信終了
    self.R = b'\x52' #通信変換基板のレジスタ読む
    self.W = b'\x57' #通信変換基板のレジスタ書く
    self.I = b'\x49' #GPIO読む
    self.O = b'\x4F' #GPIO書く
    self.Z = b'\x5A' #電源切る

    #ゲイン引数(測定範囲とか変えたかったら適宜変える)
    self.gain = 'x16'

    print("Loadcell UART BRIDGE Connected")

  def readRegister(self, address):
    self.ser.flushInput()
    send = (self.R + address + self.P)

    self.ser.write(send)

    value = ''
    while value == '' or value == b'':
      value = self.ser.read()

    if value == '' or value == b'':
      raise Exception('readRegister: No value Exception')

    return ord(value)

  def writeRegister(self, address, data):
    self.ser.flushInput()

    self.ser.write(self.W + address + data + self.P)

    if self.readRegister(address) == data:
      print('Write to register was successful')
    else:
      raise Exception('Register data mismatch from written value.')

  def write_byte_data(self, address, reg, value):

    packet = self.S + self.write_address + b'\x02' + reg + value + self.P

    #送信パケット確認用
    #print(packet.hex())

    w = self.ser.write(packet)


  def write_byte(self, value):
    packet = self.S + self.write_address + b'\x01' + value + self.P

    #送信パケット確認用
    #print(packet.hex())

    w = self.ser.write(packet)

  def read_byte(self, reg):
    packet = self.S + self.write_address + b'\x01' + reg  + self.S + self.read_address + b'\x01' + self.P

    self.ser.flushInput()
    w = self.ser.write(packet)

    value = ''
    while value == '' or value == b'': #we must wait for incoming, it should take long time
      value = self.ser.read()

    return ord(value)

  def read_byte_data(self, reg):

    self.write_byte(b'\x00')
    return self.read_byte(reg)


  #I2Cの接続が正常かチェック
  def connect_check(self):
    connect = hex(self.readRegister(b'\x0A'))
    if connect == "0xf0":
      print("I2C Connect")

    else:
      raise Exception('I2C Connect Error')

  #ゲイン設定値の保存用
  def gain_make(self, base_gain):
    print("base_gain is",base_gain)
    if base_gain == 'x128':
      #3.3V電源,ゲイン128倍に設定
      self.write_byte_data(self.address, b'\x01', b'\x27')

      #オフセットフィルタ調整値
      self.offset03 = 0x00
      self.offset04 = 0x40
      self.offset05 = 0x00

      #ゲインフィルタ調整値
      self.gain06 = 0x01
      self.gain07 = 0x00
      self.gain08 = 0x00
      self.gain09 = 0x00

    #0~1071.13gで校正済
    elif base_gain == 'x16':
      #3.3V電源,ゲイン16倍に設定
      self.write_byte_data(self.address, b'\x01', b'\x24')

      #オフセットフィルタ調整値
      self.offset03 = 0x00
      self.offset04 = 0x03
      self.offset05 = 0xE8

      #ゲインフィルタ調整値
      self.gain06 = 0x0F
      self.gain07 = 0x00
      self.gain08 = 0x00
      self.gain09 = 0x00

    elif base_gain == 'x1':
      #3.3V電源,ゲイン1倍に設定
      self.write_byte_data(self.address, b'\x01', b'\x20')

      #オフセットフィルタ調整値
      self.offset03 = 0x80
      self.offset04 = 0x00
      self.offset05 = 0xC8

      #ゲインフィルタ調整値
      self.gain06 = 0xFF
      self.gain07 = 0x00
      self.gain08 = 0x00
      self.gain09 = 0x00

    else:
      raise Exception('Unknown Gain')

    #オフセット値書き込み
    self.write_byte_data(self.address, b'\x02', b'\x32')
    self.write_byte_data(self.address, b'\x03', self.offset03.to_bytes(1, 'big'))
    self.write_byte_data(self.address, b'\x04', self.offset04.to_bytes(1, 'big'))
    self.write_byte_data(self.address, b'\x05', self.offset05.to_bytes(1, 'big'))

    #ゲイン書き込み
    self.write_byte_data(self.address, b'\x02', b'\x33')
    self.write_byte_data(self.address, b'\x06', self.gain06.to_bytes(1, 'big'))
    self.write_byte_data(self.address, b'\x07', self.gain07.to_bytes(1, 'big'))
    self.write_byte_data(self.address, b'\x08', self.gain08.to_bytes(1, 'big'))
    self.write_byte_data(self.address, b'\x09', self.gain09.to_bytes(1, 'big'))


  #センサ準備
  def power_on(self):
    #レジスタのリセット(RR=1)
    self.write_byte_data(self.address, b'\x00', b'\x01')
    print("Register reset")

    #(RR=0,PUD=1)にして200msec以上待つ
    self.write_byte_data(self.address, b'\x00', b'\x0E')
    time.sleep(1)
    print(hex(self.read_byte_data(b'\x00')))
    time.sleep(1)

    #構成の設定
    self.write_byte_data(self.address, b'\x00', b'\xAE')
    self.write_byte_data(self.address, b'\x15', b'\x30')

    #ゲインの調整
    self.gain_make(self.gain)

    #通信レート設定(80SPS)
    self.write_byte_data(self.address, b'\x02', b'\x30')

    time.sleep(1)
    #読み出しスタート
    self.write_byte_data(self.address, b'\x00', b'\xBE')
    print("Loadcell Ready!!")

  #上位2byteを読む(下位のデータはノイズ影響大)
  def single_read(self):

    data1 = self.read_byte(b'\x12')
    data2 = self.read_byte(b'\x13')

    data_row = int.from_bytes(data1.to_bytes(1, 'big') + data2.to_bytes(1, 'big'), 'big',signed='False')

    if self.gain == 'x16':
      gf_data = (data_row * 0.056594047 - 2.234957563)
      gf_data = gf_data * (gf_data > 0)

    else:
      raise Exception('It is not calibrated yet.')
    return gf_data

  #サブプロセス準備用，一度シリアル接続を切る
  def sub_ready(self):
    self.ser.close()

  #サブプロセスで並列処理する用
  #共有メモリ変数を更新し続ける
  @classmethod
  def sub_loop(cls, ser_port,I2C_address,ser_flag, shear_force):
    ser = serial.Serial(ser_port, baudrate=9600,bytesize=8, parity='N', stopbits=1, timeout=0.01)

    #I2Cデバイスのアドレス定義
    read_address =  ((I2C_address << 1) | 0x01).to_bytes(1,'big') #0x55
    write_address = ((I2C_address << 1) & 0xfe).to_bytes(1,'big')  #0x54
    address = I2C_address.to_bytes(1, 'big')

    #各種コマンド
    S = b'\x53' #I2C通信開始
    P = b'\x50'  #I2C通信終了

    while ser_flag.value:
      #上位1バイト目読む
      packet = S + write_address + b'\x01' + b'\x12'  + S + read_address + b'\x01' + P
      ser.flushInput()
      w = ser.write(packet)
      data1 = ''
      while data1 == '' or data1 == b'': #we must wait for incoming, it should take long time
        data1 = ser.read()

      #上位2バイト目読む
      packet = S + write_address + b'\x01' + b'\x13'  + S + read_address + b'\x01' + P
      ser.flushInput()
      w = ser.write(packet)
      data2 = ''
      while data2 == '' or data2 == b'': #we must wait for incoming, it should take long time
        data2 = ser.read()

      #上位2バイトをくっつけた生データ
      #data_row = int.from_bytes(data1.to_bytes(1, 'big') + data2.to_bytes(1, 'big'), 'big',signed='False')
      data_row = int.from_bytes(data1 + data2, 'big',signed='False')

      #生データを[gf]に変換，変換式は適宜実験値を入力する
      gf_data = (data_row * 0.056594047 - 2.234957563)
      #共有メモリ変数更新
      shear_force.value = gf_data * (gf_data > 0)
    ser.close()



#荷重計(gf2000)を扱うクラス,引数はつながっているCOMポート
class gf2000:
  def __init__(self, ser_port):
    self.ser = serial.Serial(ser_port, baudrate = 9600,bytesize=serial.SEVENBITS,parity = serial.PARITY_EVEN)
    print("Normal force loadcell Ready!!")

  #サブプロセスで動かすためには一度接続を切らないといけない
  def sub_ready(self):
    self.ser.close()

  #テスト用,数値を一度だけ読んで出力
  def single_read(self):
    line = ''
    while line == '' or line == b'': #we must wait for incoming, it should take long time
      line = self.ser.readline()
      #print(line)
    line = line.decode('utf-8')
    line = line[-9:-2]
    normal_force = float(line)
    return normal_force

  #サブプロセスで並列処理する用
  #共有メモリ変数を更新し続ける
  @classmethod
  def sub_loop(cls, ser_port, ser_flag, normal_force):
    ser=serial.Serial(ser_port, baudrate = 9600,bytesize=serial.SEVENBITS,parity = serial.PARITY_EVEN)
    while ser_flag.value:
      line = ""
      line = ser.readline()
      line = line.decode('utf-8')
      line = line[-9:-2]
      normal_force.value = float(line)

    ser.close()




class graphmake:
  def __init__(self):
    #---------------------------初期処理-----------------------------------------------
    nowdir = os.path.dirname(__file__)  #プログラムのあるディレクトリを参照
    print("now_directry:",nowdir)
    os.chdir(nowdir)  #作業ディレクトリをプログラムのあるディレクトリに
    os.chdir('..')
    #---------------------------ここまで初期処理---------------------------------------------

    #-----------------------何Nの力まで測定するか選択---------------------------------
    print("何Nまで測定しますか(5か10の二択です．)")
    self.Fz_str = input()  #str型の力,画像の保存名などに利用
    self.Fz = int(self.Fz_str)  #int型の力,グラフの計算に利用
    if self.Fz == 5:
      self.force_path = "record0-5xyz"

    elif self.Fz ==10:
      self.force_path = "record0-10xyz"

    else:
      print("error")
      sys.exit()
    #-------------------------ここまで力の選択---------------------------------------

    #-------------------------剪断力角度の選択(右方向を0degとして±180deg)-------------
    print("-180~180の範囲で剪断方向角度を入力してください(例:-45)")
    self.degree_str = input()
    self.theta = math.radians(int(self.degree_str)) #角度のラジアン,剪断力の分解に利用
    #-------------------------ここまで角度選択--------------------------------------


    #-------------------------保存パスの名前----------------------------------------
    print("名前と測定回数を英字半角で入力してください(例:watanabe1)")
    self.dirname = input()  #名前の入力
    #-----------------------------------------------------------------------------

    #-------------------------保存パスを作成---------------------------------------

    #./被験者名/剪断角度となるパス
    self.save_dir = "./datas/"+self.force_path +"/" + self.dirname+"/" + self.degree_str + "deg"
    os.makedirs(self.save_dir) #上記のパスを持つディレクトリ作成

    #./被験者名/剪断角度/datalog.csvとなるパス(ここに測定値を記入)
    self.datalog_path = self.save_dir + "/datalog.csv"
    self.namelist_path = "./datas/"+self.force_path + "/namelist.csv"

    #-------------------------ネームリストに書き込み---------------------------------
    #./force_pathにあるcsvファイルに各被験者のdatalog.csvを書き込む(機械学習時にここから全データを参照するため)
    self.name_csv = open(self.namelist_path,"a",newline="")
    self.name_writing = csv.writer(self.name_csv)
    record_path = self.force_path +"/" + self.dirname+"/" + self.degree_str + "deg"+ "/datalog.csv"
    self.name_writing.writerow([record_path,self.degree_str])
    self.name_csv.close()
    #------------------------ここまでネームリスト書き込み------------------------------
    self.data_csv = open(self.datalog_path,"w",newline="")
    self.data_writing = csv.writer(self.data_csv)

    #------------------------測定値を格納するcsvの準備----------------------------------

    #-----------------------------グラフ用パラメータ----------------------------------------
    #せん断力の測定範囲で一部パラメータを分ける
    if self.Fz==5:
      self.mu = 0.9                   #摩擦係数
      self.fz = 0.03                  #垂直力振動数[Hz]
      self.T_num = 6                  #測定周期

    elif self.Fz==10:
      self.mu = 0.5                   #摩擦係数
      self.fz = 0.025                  #垂直力振動数[Hz]
      self.T_num = 5                  #測定周期

    self.Fr = self.Fz*self.mu
    self.now_Fz = 0.0
    self.now_Fz = 0.0
    self.t_rest = 10.0  #[sec]　予備時間
    self.t_flex = 0.0               #曲げ伸ばしの時間[sec]

    self.fr = 0.025                 #剪断力振動数[Hz]
    self.T_count = 0
    self.T = 1/self.fz
    #self.rectime = 1
    self.rectime = self.T * self.T_num  #測定時間
    self.recstart  = self.t_rest
    self.recfinish = self.t_rest +self.t_flex + self.rectime
    self.graphtime = self.rectime+2*self.t_rest
    self.frontT = 3.0  #x軸表示範囲
    self.backT = 1.0
    self.Frange = 0.2  #拡張値域
    self.N2gf = 101.972
    self.datanum = 0
    self.error_r = 0.5  #誤差1N以内のデータを保存
    #----------------------------------------------------------------------------------------



    #------------------------画像初期処理-------------------------
    self.roix,self.roiy=260,165   #左上座標
    self.w,self.h=140,155         #幅,高さ
    self.capture = cv2.VideoCapture(0)
    if self.capture.isOpened() is False:
      raise ("IO Error")
    #-----------------------------------------------------------



    #グラフ生成
    self.fig = plt.figure(figsize=(12,6))
    self.fig.subplots_adjust(wspace=0.5)
    self.axL = self.fig.add_subplot(121,xlabel = "width",ylabel = "hight")
    self.axR = self.fig.add_subplot(122,xlabel = "xyForce",ylabel = "zForce" )


    #--------------------------axR設定-------------------------------------------


    self.axR.set_xlim(-0.5,self.Fz+0.5)
    self.axR.set_ylim(-0.5,self.Fz+0.5)
    self.axR.xaxis.set_ticks_position("bottom") # 目盛りは軸の下

    self.t = 0.00
    self.rec_t = 0.00
    #self.num, self.xdata, self.ydata, self.zdata =[], [], [], []
    self.line, = self.axR.plot([], [], color='green', label="example")
    self.line_upper, = self.axR.plot([],[],color='red')
    self.line_lower, = self.axR.plot([],[],color='red')
    #self.rec_range = patches.Circle(xy=(0,0), radius=self.error_r,ec='r',fill = False)
    #self.axR.add_patch(self.rec_range)
    self.now_F, = self.axR.plot([],[],color='red',marker=".",markersize=10,label = "now_ex")
    self.rec1, = self.axR.plot([], [], color='red' , marker='.', markersize=10,label='before')
    self.rec2, = self.axR.plot([], [], color='blue', marker='.', markersize=10,label='recording')
    #-------------------------------------------------------------------------------

    #-------------------------axL設定------------------------------
    self.image_init0 = np.zeros((self.h, self.w), dtype = "uint8")
    self.image_init1 = np.ones((self.h, self.w), dtype = "uint8")*255
    #vmin.vmaxをちゃんとせっていしないと沼る
    self.image_plt = self.axL.imshow(self.image_init0, animated=True,cmap="gray",vmin=0,vmax=255)
    #--------------------------------------------------------------

    #-----------------------教示グラフ-------------------------------------------------------------------------------------------------
    self.t_line = np.arange(0, 5,0.001)
    self.Fz_line =self.Fz/2*(1-np.cos(2*np.pi*self.fz*self.t_line))
    self.Fr_line =self.Fz_line*self.mu / (self.T_num - 1) * (1*(self.t_line > self.T) + 1*(self.t_line > self.T * 2) + 1*(self.t_line > self.T * 3) + 1*(self.t_line > self.T * 4) + 1*(self.t_line > self.T * 5))

    self.y_ex = -(self.Fz/2*(np.cos(2*np.pi*self.fz*(self.t_line-(self.t_rest+self.t_flex))))-self.Fz/2)*(self.t_line > self.t_rest + self.t_flex)
    self.y_upper = self.y_ex + 0.5
    self.y_lower = self.y_ex-0.5
    self.now_Fz = 0.0
    self.now_Fr = 0.0

  #-----------キー入力----------------------------
  def onkey(self,event):

    if event.key == 'escape':
      print("esc")
      ser_flag.value = False
      self.data_csv.close()
      plt.close(event.canvas.figure)

    if event.key == 'r':
      print("r")
      rec_flag.value = True
      print(rec_flag.value)
      self.graphstart = time.perf_counter()


    if str.isdecimal(event.key):
      pass
  #---------ここまでキー入力----------------------


  def updateframe(self,dum):
    t0 =time.perf_counter()
    #左側グラフ(カメラ)
    ret, base = self.capture.read()

    #画像を切り出し
    ROI = base[ self.roiy:self.roiy+self.h , self.roix:self.roix+self.w ]

    #グレースケールに変換(緑だけ抽出)
    img_blue_c1, img_green_c1, img_red_c1 = cv2.split(ROI)
    gray = img_green_c1

    #ガウシアンフィルタ(ノイズ除去)
    gau = cv2.GaussianBlur(gray, ksize=(5,5), sigmaX=0)

    #ヒストグラム平坦化
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
    hist = clahe.apply(gau)

    #左側のグラフに動画表示
    self.image_plt.set_array(hist)

    self.Fz_line =self.Fz/2*(1-np.cos(2*np.pi*self.fz*self.t_line))
    self.Fr_line =self.Fz_line*self.mu / (self.T_num - 1)* (1*(self.t_line > self.T) + 1*(self.t_line > self.T * 2) + 1*(self.t_line > self.T * 3) + 1*(self.t_line > self.T * 4) + 1*(self.t_line > self.T * 5))

    #右側のグラフに教示線載せる
    self.line.set_data(self.Fr_line, self.Fz_line)
    #self.line_upper.set_data(self.t_line, self.y_upper)
    #self.line_lower.set_data(self.t_line, self.y_lower)

    #荷重計の取った値[g]を[gf]として[N]に変換(ここ並列処理にすると40msec高速化)
    Fz =  normal_force.value/ self.N2gf
    #Fz =  normal_loadcell.single_read() / self.N2gf

    #せん断力(ここ並列処理にすると20msec高速化)
    #Fr = shear_force.value  / self.N2gf
    Fr = shear_force.value  / self.N2gf

    dum_Fz = random.uniform(0,self.Fz)
    dum_Fr = random.uniform(0,self.Fz)


    #測定状態(rec_flag==True)
    if rec_flag.value:
      self.t = time.perf_counter() - self.graphstart
      self.rec_t = self.t - self.t_rest
      #print(self.rec_t)

      #測定前予備時間
      if self.rec_t < 0:
        print(self.rec_t)
        self.t_line = np.arange(0.0, 5.0,0.001)


      #測定中
      elif self.rec_t > 0 and self.rec_t < self.rectime:
        self.t_line = np.arange(self.rec_t, self.rec_t +5.0,0.001)
        print("recording","time",round(self.rec_t,3),"Fz",round(Fz,2),"Fr",round(Fr,2))

        self.now_Fz =self.Fz/2*(1-np.cos(2*np.pi*self.fz*self.rec_t))
        self.now_Fr =self.now_Fz*self.mu / (self.T_num - 1)* (1*(self.rec_t > self.T) + 1*(self.rec_t > self.T * 2) + 1*(self.rec_t > self.T * 3) + 1*(self.rec_t > self.T * 4) + 1*(self.rec_t > self.T * 5))

        self.error = np.sqrt(np.power(self.now_Fz - Fz,2) + np.power(self.now_Fr - Fr,2))
        if self.error < self.error_r:
          framename = self.save_dir +"/"+ str(self.datanum) + '.png'  #保存する画像のパスと名前
          cv2.imwrite(framename, hist)                             #画像を保存
          self.data_writing.writerow([framename,Fz,Fr])    #測定値をcsvに書き込み
          #print('The number of data = ' + str(datanum) )
          self.datanum += 1

        #リミッター解除(全データ記録)
        else:
          framename = self.save_dir +"/"+ str(self.datanum) + '.png'  #保存する画像のパスと名前
          cv2.imwrite(framename, hist)                             #画像を保存
          self.data_writing.writerow([framename,Fz,Fr])    #測定値をcsvに書き込み
          #print('The number of data = ' + str(datanum) )
          self.datanum += 1

      #測定終了
      else:
        self.data_csv.close()
        ser_flag.value = False
        print("press esc")
        self.ani.event_source.stop()

      #self.rec_range = patches.Circle(xy=(self.now_Fr,self.now_Fz), radius=self.error_r,ec='r',fill = False)
      #self.axR.add_patch(self.rec_range)
      self.rec2.set_data(Fr, Fz)
      self.now_F.set_data(self.now_Fr,self.now_Fz)
      return self.rec2,self.image_plt,self.line,self.now_F,
      #-----------ここまで測定処理-----------------------------

    #測定待機状態(rec_flag==False)
    else:
      self.rec1.set_data(Fr, Fz)
      return self.rec1, self.image_plt, self.line,



  def animation(self):
    self.ani = animation.FuncAnimation(self.fig, self.updateframe, interval=0, blit=True)
    self.cid = self.fig.canvas.mpl_connect('key_press_event', self.onkey)
    plt.tight_layout()
    plt.show()

#ここからサブプロセス用関数
def sub_normal(arg,**kwargs):
  return gf2000.sub_read(*arg,**kwargs)


def sub_shear(ser_flag, shear_gf):
  ser_flag = ser_flag.value
  while ser_flag:
    shear_gf = shear_loadcell.single_read()
    ser_flag = ser_flag.value
#ここまでサブプロセス用関数


if __name__ == "__main__":
  #--------メモリ共有変数-------------
  normal_force = Value('f', 0.00)
  shear_force = Value('f', 0.00)
  ser_flag = Value('b', True)   #シリアル通信フラグ(Trueで荷重計，MD共にループ開始)
  rec_flag = Value('b',False)   #測定フラグ(これがTrueの間測定)

  #-----------------------------------

  #ロードセル測定準備
  xy_port = "COM7"
  xy_address = 0x2A
  shear_loadcell = SC800IM700(xy_port, xy_address)
  #shear_loadcell.connect_check()
  shear_loadcell.power_on()
  shear_loadcell.sub_ready()

  #荷重計測定準備
  z_port = "COM6"
  normal_loadcell = gf2000(z_port)
  normal_loadcell.sub_ready()

  """
  サブプロセス開始(各種通信)
  並列処理したい関数がクラス内の関数(メソッド)の場合エラーが起きる
  その場合，メソッドをクラスメソッドとして定義してやると動かせる
  """
  sub_z = Process(target=gf2000.sub_loop,args=[z_port,ser_flag,normal_force])
  sub_z.start()
  sub_xy = Process(target=SC800IM700.sub_loop,args=[xy_port,xy_address,ser_flag,shear_force])
  sub_xy.start()

  graph = graphmake()

  #グラフ表示
  graph.animation()



  #-----各サブプロセス終了
  if ser_flag ==False:
    sub_z.join()
    sub_xy.join()
    pass
