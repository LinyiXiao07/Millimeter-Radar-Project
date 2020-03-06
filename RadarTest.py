import serial
import time
import re
import RPi.GPIO as GPIO

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

BuzzerPin = 17


def main():
  # Main program block
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7

  GPIO.setup(BuzzerPin, GPIO.OUT)
  GPIO.output(BuzzerPin, GPIO.LOW)

  # Initialise display
  lcd_init()
  setup(Buzzer)

  while True:
        print(" 1 ==>  24GHz Radar")
        print(" 2 ==> 122GHz Radar")
        lcd_string("Enter your op",LCD_LINE_1)
        radarFrontEnd= input('Please enter your option?\r\n')
        lcd_string("Enter:%s"%radarFrontEnd,LCD_LINE_2)
        if radarFrontEnd == '1':
            print('Start to set 24GHz Radar Configuration')
            send24Gcmd()
            printData()
        elif radarFrontEnd == '2':
            print('Start to set 122GHz Radar Configuration')
            send122Gcmd()
            printData()

    # Send some test
#    lcd_string("Target Distance",LCD_LINE_1)
#    lcd_string("xxxx mm",LCD_LINE_2)

#    time.sleep(3) # 3 second delay

    # Send some text
#    lcd_string("1234567890123456",LCD_LINE_1)
#    lcd_string("abcdefghijklmnop",LCD_LINE_2)

#    time.sleep(3) # 3 second delay

    # Send some text
#    lcd_string("Basemu.com",LCD_LINE_1)
#    lcd_string("Welcome",LCD_LINE_2)

#    time.sleep(3)

    # Send some text
#    lcd_string("Welcome to",LCD_LINE_1)
#    lcd_string("Basemu.com",LCD_LINE_2)

#    time.sleep(3)

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def lcd_clear():
   lcd_byte(0x01, LCD_CMD)
   lcd_byte(0x02, LCD_CMD)

###############################################################################

Buzzer = 17    # pin11 GPIO 17

def setup(pin):
	global BuzzerPin
	BuzzerPin = pin
	#GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location


def off():
	GPIO.output(BuzzerPin, GPIO.LOW)

def on():
	GPIO.output(BuzzerPin, GPIO.HIGH)

def beep(x):
	on()
	time.sleep(x)
	off()
	time.sleep(x)

def loop():
	while True:
		beep()

def destroy():
	GPIO.output(BuzzerPin, GPIO.HIGH)
	GPIO.cleanup()                     # Release resource


###############################################################################
class Configuration:
    def __init__(self,system,radarFrontend,pll,basebandSetup):
        self.systemConfiguration = system
        self.radarFrontendConfiguration = radarFrontend
        self.pllConfiguration = pll
        self.basebandSetup = basebandSetup

    def  sendCmdMsg(self):
        ser.write(bytes(self.systemConfiguration,encoding='utf8')+b'\r\n')
        time.sleep(0.2)
        ser.write(bytes(self.radarFrontendConfiguration,encoding='utf8')+b'\r\n')
        time.sleep(0.2)
        ser.write(bytes(self.pllConfiguration,encoding='utf8')+b'\r\n')
        time.sleep(0.2)
        ser.write(bytes(self.basebandSetup,encoding='utf8')+b'\r\n')
        time.sleep(0.2)

    def __str__(self):
        return 'System-{} Front End-{} Pll-{} Baseband-{}'.format(self.systemConfiguration,self.radarFrontendConfiguration,self.pllConfiguration,self.basebandSetup)

###############################################################################
serialPort="/dev/ttyACM0"   #Seroal port "/dev/ttyACM0"
baudRate=230400     #BaudRate
ser=serial.Serial(serialPort,baudRate,timeout=0.5)
print( "Parameters Setting：Port= %s ，BaudRate= %d"%(serialPort,baudRate))

###############################################################################
radar24G = Configuration("!S000049BA","!F004059D8","!P00000BB8","!BBAB5C2EC")
#radar24G.sendCmdMsg()

radar122G = Configuration("!S000049BA","!F0201D1A0","!P000019C8","!BB484C325")
#radar122G.sendCmdMsg()

###############################################################################

def send24Gcmd():
    radar24G.sendCmdMsg()
    print('\nThe systemConfiguration has beed set.')
    print('\nThe radarFrontendConfiguration has beed set.')
    print('\nThe pllConfiguration has beed set.')
    print('\nThe basebandSetup has beed set.\r\n')

def send122Gcmd():
    radar122G.sendCmdMsg()
    print('\nThe systemConfiguration has beed set.')
    print('\nThe radarFrontendConfiguration has beed set.')
    print('\nThe pllConfiguration has beed set.')
    print('\nThe basebandSetup has beed set.\r\n')

###############################################################################

dataMatch = 0
list_empty = []

def rawData():
    while 1:
        data = ser.readline()
        dataEncode = data.decode('latin-1',"ignore")
        print(dataEncode)

def objectInfo():
    while 1:
        data = ser.readline()
        #decode the raw data with 'latin-1' to ASCII string/ignore the error
        ascData = data.decode('latin-1',"ignore")
        #pre compile regular expression for target information code format
        preCpl = re.compile('^!T5.*?\r\n$') #matching rule
        #delay0.5
        time.sleep(0.1)
        #find all the matching part in ascData string and save it to new variable[dataMatch]
        dataMatch = re.findall(preCpl,ascData)
        #if the data list is not Nonetype, it will save to a new variable
        if dataMatch != list_empty: # matching list is not empty. Filter the available data
            validData = dataMatch #save available data to a new variable
            #move the string out of the list
            targetInfo = validData[0]
            #Format and Gain
            print('***************************************')
            print('\r\nstart: ',targetInfo[0])
            print('identifier: ',targetInfo[1])
            print('format: (H)',targetInfo[2])
            print('gain: (D)', ord(targetInfo[3])-174)
            #Target 0
            print('\r\nblock 0  : ',targetInfo[4:18])
            print('lenth01: ', len(targetInfo[4:18]))
            print('Target#:',targetInfo[4])
            print('Distance',int(str(targetInfo[5:9]),16))
            print('Mag : ', ord(targetInfo[9])-174)
            print('Phi : ', signedH2I(targetInfo[10:14])-32768)
            #Target 1
            print('\r\nblock 1  : ',targetInfo[18:32])
            print('lenth02: ', len(targetInfo[18:32]))
            print('Target#:',int(targetInfo[18],16))
            print('Distance',int(str(targetInfo[19:23]),16))
            print('Mag : ', ord(targetInfo[23])-174)
            print('Phi : ', signedH2I(targetInfo[24:28])-32768)
            #Target 2
            print('\nblock 2  : ',targetInfo[32:46])
            print('lenth03: ', len(targetInfo[32:46]))
            print('Target#:',int(targetInfo[32],16))
            print('Distance',int(str(targetInfo[33:37]),16))
            print('Mag : ', ord(targetInfo[37])-174)
            print('Phi : ', signedH2I(targetInfo[38:42])-32768)
            #Target 3
            print('\nblock 3  : ',targetInfo[46:60])
            print('lenth04: ', len(targetInfo[46:60]))
            print('Target#:',int(targetInfo[46],16))
            print('Distance',int(str(targetInfo[47:51]),16))
            print('Mag : ', ord(targetInfo[51])-174)
            print('Phi : ', signedH2I(targetInfo[52:56])-32768)
            #Target 4
            print('\nblock 4  : ',targetInfo[60:74])
            print('lenth04: ', len(targetInfo[60:74]))
            print('Target#:',int(targetInfo[60],16))
            print('Distance',int(str(targetInfo[61:65]),16))
            print('Mag : ', ord(targetInfo[65])-174)
            print('Phi : ', signedH2I(targetInfo[66:70])-32768)


def distanceInfo():
    lcd_clear()
    while 1:
        data = ser.readline()
        ascData = data.decode('latin-1',"ignore")
        preCpl = re.compile('^!T5.*?\r\n$')
        #time.sleep(0.05)
        dataMatch = re.findall(preCpl,ascData)
        if dataMatch != list_empty: # 存在值即为真
            validData = dataMatch
            targetInfo = validData[0]
            print('Distance',int(str(targetInfo[19:23]),16))
            dist = int(str(targetInfo[19:23]),16)
            if  dist == 0:
                off()
            elif dist > 40 and dist < 500:
                beep(0.02)
            elif dist >= 500 and dist < 1000:
                beep(0.04)
            elif dist >= 1000 and dist < 2000:
                beep(0.08)
            elif dist >= 2000:
                beep(1)
            lcd_string("Target Distance",LCD_LINE_1)
            lcd_string("%d mm"%dist,LCD_LINE_2)

def signedH2I(value):
    lenth = len(value)
    decData = int(str(value),16)
    if decData > 2 ** (lenth-1)-1 and decData != None:
       decData = 2 ** lenth-decData
       decData = 0 - decData
       if decData != None:
          return decData
       else:
          return decData
    return decData

###############################################################################

def printData():
    print("      1 ==> Object info")
    print("      2 ==> Raw data stream from radar")
    print("      3 ==> Target 1's Distance")
    print("Any Key ==> Exit")

    lcd_string("Enter your op",LCD_LINE_1)
    choice = input('Please enter your option?\r\n')
    lcd_string("Enter: %s"%choice,LCD_LINE_2)
    if choice == '1':
        lcd_string("object info",LCD_LINE_1)
        time.sleep(1)
        print('Start to give object info')
        objectInfo()
    elif choice == '2':
        lcd_string("raw data",LCD_LINE_1)
        time.sleep(1)
        print('Start to print raw data stream')
        rawData()
    elif choice == '3':
        lcd_string("Target",LCD_LINE_1)
        time.sleep(1)
        print('Start to print Target 1 Distance information')
        distanceInfo()
    else :
        print('close the program')
        ser.close()

###############################################################################


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        lcd_byte(0x01, LCD_CMD)
        lcd_string("Goodbye!",LCD_LINE_1)
        destroy()
        GPIO.cleanup()