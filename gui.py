from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtlet import L, MapWidget
from math import cos, asin, sqrt, pi
from datetime import datetime
import os
import re
import time
import threading
import queue
import serial
import serial.tools.list_ports

class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # Read and Write threads
        self.readThread = threading.Thread()
        self.writeThread = threading.Thread()
        self.writeQueue = queue.Queue()
        self.readState = 1 # read thread state
        self.loraCommands = ['AT\r\n', 'AT+VER?\r\n', 'AT+UID?\r\n', 'AT+BAND?\r\n', 'AT+NETWORKID?\r\n',
            'AT+ADDRESS?\r\n', 'AT+PARAMETER?\r\n', 'AT+IPR?\r\n']
        self.message = ['NA','','NA','NA',''] # LoRa message structure
        self.currentPort = ' ' 
        self.ports = [' '] # list of serial ports
        self.portsFullName = [' '] # verbose list of serial ports
        for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
            self.ports.append(port)
            self.portsFullName.append(port + ' - ' + hwid)
        self.connected = False # serial connection state
        self.serialPort = serial.Serial()
        self.setWindowIcon(QtGui.QIcon('images/icon.png'))
        self.setWindowTitle('Ground Station')
        # create logs folder/files
        if(not os.path.exists('logs')): os.makedirs('logs')
        self.coordinatesFile = open('logs/coordinates.txt', 'a')  
        self.telemetryFile = open('logs/telemetry.txt', 'a')  
        self.destinationFile = open('logs/destination.txt', 'a')  
        self.receivedFile = open('logs/received.txt', 'a')  
        # Fonts
        self.font14 = QtGui.QFont()
        self.font14.setPointSize(14)
        self.font16 = QtGui.QFont()
        self.font16.setPointSize(16)
        # Layouts
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.CMTabUI(), 'Controls/Map')
        tabs.addTab(self.SHTabUI(), 'Settings/Help')
        self.layout.addWidget(tabs)

    def CMTabUI(self):
        self.coordinate = [28.602691, -81.200102]
        self.originalCoordinate = [28.602691, -81.200102]
        self.destinationCoordinate = [28.602691, -81.200102]
        CMTab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        self.layout.setSpacing(0)
        self.initLayout = QtWidgets.QGridLayout()
        self.initLayout.setVerticalSpacing(10)
        self.initLayout.setHorizontalSpacing(10)
        self.manualLayout = QtWidgets.QGridLayout()
        self.manualLayout.setVerticalSpacing(10)
        self.manualLayout.setHorizontalSpacing(10)
        self.blindDriveLayout = QtWidgets.QGridLayout()
        self.blindDriveLayout.setVerticalSpacing(10)
        self.blindDriveLayout.setHorizontalSpacing(10)
        self.telemetryLayout = QtWidgets.QGridLayout()
        self.telemetryLayout.setVerticalSpacing(10)
        self.telemetryLayout.setHorizontalSpacing(10)
        self.portText = QtWidgets.QLabel()
        self.portText.setText('Port')
        self.portText.setFont(self.font16)
        self.portText.setAlignment(QtCore.Qt.AlignCenter)
        self.portList = QtWidgets.QComboBox()
        self.portList.addItems(self.portsFullName)
        self.portList.activated.connect(self.switchPort)
        self.controlText = QtWidgets.QLabel()
        self.controlText.setText('Control Mode')
        self.controlText.setFont(self.font16)
        self.controlText.setAlignment(QtCore.Qt.AlignCenter)
        self.sentText = QtWidgets.QLabel()
        self.sentText.setText('Last Command Sent')
        self.sentText.setFont(self.font16)
        self.sentText.setAlignment(QtCore.Qt.AlignCenter)
        self.controlList = QtWidgets.QComboBox()
        self.controlList.addItems(['', 'Blind Drive', 'Manual'])
        self.controlList.activated.connect(self.switchControl)
        self.controlList.setDisabled(True)
        self.sent = QtWidgets.QLabel()
        self.sent.setText('NA')
        self.sent.setAlignment(QtCore.Qt.AlignCenter)
        self.address = QtWidgets.QLabel()
        self.address.setText('Rover\'s Address: NA')
        self.address.setFont(self.font14)
        self.address.setAlignment(QtCore.Qt.AlignLeft)
        self.data = QtWidgets.QLabel()
        self.data.setText('Data: NA')
        self.data.setFont(self.font14)
        self.data.setAlignment(QtCore.Qt.AlignLeft)
        self.signal = QtWidgets.QLabel()
        self.signal.setText('Signal: NA')
        self.signal.setFont(self.font14)
        self.signal.setAlignment(QtCore.Qt.AlignLeft)
        self.forward = QtWidgets.QPushButton()
        self.forward.clicked.connect(lambda: self.buttonPressed('w'))
        self.forward.setAutoRepeat(True)
        self.forward.setIcon(QtGui.QIcon('images/up.png'))
        self.forward.setIconSize(QtCore.QSize(50,50))
        self.left = QtWidgets.QPushButton()
        self.left.clicked.connect(lambda: self.buttonPressed('a')) 
        self.left.setAutoRepeat(True)
        self.left.setIcon(QtGui.QIcon('images/left.png'))
        self.left.setIconSize(QtCore.QSize(50,50))
        self.reverse = QtWidgets.QPushButton()
        self.reverse.clicked.connect(lambda: self.buttonPressed('s')) 
        self.reverse.setAutoRepeat(True)
        self.reverse.setIcon(QtGui.QIcon('images/down.png'))
        self.reverse.setIconSize(QtCore.QSize(50,50))
        self.right = QtWidgets.QPushButton()
        self.right.clicked.connect(lambda: self.buttonPressed('d')) 
        self.right.setAutoRepeat(True)
        self.right.setIcon(QtGui.QIcon('images/right.png'))
        self.right.setIconSize(QtCore.QSize(50,50))
        self.destLat = QtWidgets.QLineEdit()
        self.destLat.setMaxLength(10)
        self.destLat.setAlignment(QtCore.Qt.AlignLeft)
        self.destLat.setFont(self.font14)
        self.destLong = QtWidgets.QLineEdit()
        self.destLong.setMaxLength(11)
        self.destLong.setAlignment(QtCore.Qt.AlignLeft)
        self.destLong.setFont(self.font14)
        self.travelState = 1
        self.travel = QtWidgets.QPushButton('Travel')
        self.travel.clicked.connect(lambda: self.buttonPressed('t')) 
        self.travel.setDisabled(True)
        self.latText = QtWidgets.QLabel()
        self.latText.setText('Lat: ' + str(self.originalCoordinate[0]))
        self.latText.setFont(self.font14)
        self.latText.setAlignment(QtCore.Qt.AlignLeft)
        self.longText = QtWidgets.QLabel()
        self.longText.setText('Long: ' + str(self.originalCoordinate[1]))
        self.longText.setFont(self.font14)
        self.longText.setAlignment(QtCore.Qt.AlignLeft)
        self.distanceText = QtWidgets.QLabel()
        self.distanceText.setText('Distance: 0 m')
        self.distanceText.setFont(self.font14)
        self.distanceText.setAlignment(QtCore.Qt.AlignLeft)
        self.starting = QtWidgets.QPushButton('Starting Location')
        self.starting.clicked.connect(lambda: self.panTo('s')) 
        self.current = QtWidgets.QPushButton('Current Location')
        self.current.clicked.connect(lambda: self.panTo('c')) 
        self.toggle = QtWidgets.QPushButton('Toggle Map Style')
        self.toggle.clicked.connect(lambda: self.mapToggle()) 
        self.batteryText = QtWidgets.QLabel()
        self.batteryText.setText('Battery: 0 hrs')
        self.batteryText.setFont(self.font14)
        self.batteryText.setAlignment(QtCore.Qt.AlignLeft)
        self.speedText = QtWidgets.QLabel()
        self.speedText.setText('Speed: 0 km/h')
        self.speedText.setFont(self.font14)
        self.speedText.setAlignment(QtCore.Qt.AlignLeft)
        self.initLayout.addWidget(self.portText, 0, 0)
        self.initLayout.addWidget(self.controlText, 0, 1)
        self.initLayout.addWidget(self.sentText, 0, 2)
        self.initLayout.addWidget(self.portList, 1, 0)
        self.initLayout.addWidget(self.controlList, 1, 1)
        self.initLayout.addWidget(self.sent, 1, 2)
        self.initLayout.addWidget(self.address, 2, 0)
        self.initLayout.addWidget(self.data, 2, 1)
        self.initLayout.addWidget(self.signal, 2, 2)
        self.manualLayout.addWidget(self.forward, 0, 1)
        self.manualLayout.addWidget(self.left, 1, 0)
        self.manualLayout.addWidget(self.right, 1, 2)
        self.manualLayout.addWidget(self.reverse, 2, 1)
        self.blindDriveLayout.addWidget(self.destLat, 0, 0)
        self.blindDriveLayout.addWidget(self.destLong, 0, 1)
        self.blindDriveLayout.addWidget(self.travel, 0, 2)
        self.telemetryLayout.addWidget(self.latText, 1, 0)
        self.telemetryLayout.addWidget(self.longText, 1, 1)
        self.telemetryLayout.addWidget(self.batteryText, 0, 0)
        self.telemetryLayout.addWidget(self.speedText, 0, 1)
        self.telemetryLayout.addWidget(self.distanceText, 0, 2)
        self.telemetryLayout.addWidget(self.starting, 2, 0)
        self.telemetryLayout.addWidget(self.current, 2, 1)
        self.telemetryLayout.addWidget(self.toggle, 2, 2)
        self.mapLayout = QtWidgets.QVBoxLayout()
        self.mapWidget = MapWidget()
        self.map = L.map(self.mapWidget) 
        self.map.setView(self.coordinate, 18)
        self.worldMap = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
        self.darkMap = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
        self.maps = [self.worldMap, self.darkMap]
        self.currentMap = 0
        L.tileLayer(self.maps[self.currentMap], {'maxNativeZoom': 19, 'maxZoom': 25, 'noWrap': 'true'}).addTo(self.map)
        self.marker = L.marker(self.originalCoordinate)
        self.marker.bindTooltip('Rover Starting Position')
        self.marker2 = L.marker(self.coordinate)
        self.marker2.bindTooltip('Rover Current Position')
        self.destMarker = L.marker(self.originalCoordinate, options = {"opacity": 0})
        self.layerGroup = L.layerGroup()
        self.map.runJavaScript(f'{self.layerGroup.jsName}' + \
            '.addLayer(' + self.marker.jsName + ')' + \
            '.addLayer(' + self.marker2.jsName + ')' + \
            '.addLayer(' + self.destMarker.jsName + ')' + \
            '.addTo(' + self.map.jsName + ');')
        self.imageDir = os.path.join(os.getcwd(), 'images').replace('\\', '/')
        self.map.runJavaScript('var markerIcon = L.icon({iconUrl: \"' + self.imageDir + '/start.png\"});')
        self.map.runJavaScript(f'{self.marker.jsName}.setIcon(markerIcon);')
        self.map.runJavaScript('var markerIcon2 = L.icon({iconUrl: \"' + self.imageDir + '/rover.png\"});')
        self.map.runJavaScript(f'{self.marker2.jsName}.setIcon(markerIcon2);')
        self.map.runJavaScript('var markerIcon3 = L.icon({iconUrl: \"' + self.imageDir + '/marker.png\"});')
        self.map.runJavaScript('var markerIcon4 = L.icon({iconUrl: \"' + self.imageDir + '/destination.png\"});')
        self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
        self.map.runJavaScript('var startCoordinate = [[' + str(self.originalCoordinate[0]) + ',' + str(self.originalCoordinate[1]) + ']]')
        self.map.runJavaScript("var polyline = L.polyline(startCoordinate, {color: '#0077ff'}).addTo(" + self.map.jsName + ");")
        self.map.runJavaScript('polyline.bindTooltip(\"Rover\'s Path\");')
        self.map.clicked.connect(lambda x: self.setDest(x['latlng']))
        self.mapLayout.addWidget(self.mapWidget)
        self.hideControls(True, 'all')
        layout.addLayout(self.initLayout)
        layout.addLayout(self.manualLayout)
        layout.addLayout(self.blindDriveLayout)
        layout.addLayout(self.telemetryLayout)
        layout.addLayout(self.mapLayout)
        layout.addStretch()
        CMTab.setLayout(layout)
        return CMTab

    def SHTabUI(self):
        SHTab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(5)
        portLayout = QtWidgets.QFormLayout()
        loraLayout = QtWidgets.QHBoxLayout()
        loraVLayout = QtWidgets.QVBoxLayout()
        loraOptionsLayout = QtWidgets.QFormLayout()
        self.baudrateText = QtWidgets.QLineEdit()
        self.baudrateText.setAlignment(QtCore.Qt.AlignRight)
        self.baudrateText.setText('115200')
        self.baudrateText.setFixedWidth(70)
        self.bytesizeText = QtWidgets.QLineEdit()
        self.bytesizeText.setAlignment(QtCore.Qt.AlignRight)
        self.bytesizeText.setText('8')
        self.bytesizeText.setFixedWidth(70)
        self.timeoutText = QtWidgets.QLineEdit()
        self.timeoutText.setAlignment(QtCore.Qt.AlignRight)
        self.timeoutText.setText('2')
        self.timeoutText.setFixedWidth(70)
        self.spreadingFactor = QtWidgets.QLineEdit()
        self.spreadingFactor.setAlignment(QtCore.Qt.AlignRight)
        self.spreadingFactor.setText('12')
        self.spreadingFactor.setFixedWidth(70)
        self.bandwidth = QtWidgets.QLineEdit()
        self.bandwidth.setAlignment(QtCore.Qt.AlignRight)
        self.bandwidth.setText('7')
        self.bandwidth.setFixedWidth(70)
        self.codingRate = QtWidgets.QLineEdit()
        self.codingRate.setAlignment(QtCore.Qt.AlignRight)
        self.codingRate.setText('2')
        self.codingRate.setFixedWidth(70)
        self.preamble = QtWidgets.QLineEdit()
        self.preamble.setAlignment(QtCore.Qt.AlignRight)
        self.preamble.setText('5')
        self.preamble.setFixedWidth(70)
        self.gsAddress = QtWidgets.QLineEdit()
        self.gsAddress.setAlignment(QtCore.Qt.AlignRight)
        self.gsAddress.setText('101')
        self.gsAddress.setFixedWidth(70)
        self.roverAddress = QtWidgets.QLineEdit()
        self.roverAddress.setAlignment(QtCore.Qt.AlignRight)
        self.roverAddress.setText('102')
        self.roverAddress.setFixedWidth(70)
        self.networkID = QtWidgets.QLineEdit()
        self.networkID.setAlignment(QtCore.Qt.AlignRight)
        self.networkID.setText('5')
        self.networkID.setFixedWidth(70)
        self.band = QtWidgets.QLineEdit()
        self.band.setAlignment(QtCore.Qt.AlignRight)
        self.band.setText('915000000')
        self.band.setFixedWidth(70)
        self.uart = QtWidgets.QLineEdit()
        self.uart.setAlignment(QtCore.Qt.AlignRight)
        self.uart.setText('115200')
        self.uart.setFixedWidth(70)
        portLayout.addRow('Baudrate:', self.baudrateText)
        portLayout.addRow('Bytesize:', self.bytesizeText)
        portLayout.addRow('Timeout:', self.timeoutText)
        loraOptionsLayout.addRow('Spreading Factor:', self.spreadingFactor)
        loraOptionsLayout.addRow('Bandwidth:', self.bandwidth)
        loraOptionsLayout.addRow('Coding Rate:', self.codingRate)
        loraOptionsLayout.addRow('Programmed Preamble:', self.preamble)
        loraOptionsLayout.addRow('Address (Ground Station):', self.gsAddress)
        loraOptionsLayout.addRow('Address (Rover):', self.roverAddress)
        loraOptionsLayout.addRow('Network ID:', self.networkID)
        loraOptionsLayout.addRow('Band:', self.band)
        loraOptionsLayout.addRow('UART:', self.uart)
        self.allSet = QtWidgets.QPushButton('Set All')
        self.allSet.clicked.connect(lambda: self.setAll()) 
        self.setParameters = QtWidgets.QPushButton('Set Parameters')
        self.setParameters.clicked.connect(lambda: self.setParams(1)) 
        self.testLora = QtWidgets.QPushButton('LoRa Info')
        self.testLora.clicked.connect(lambda: self.loraTest()) 
        self.customCommand = QtWidgets.QLineEdit()
        self.customCommand.setAlignment(QtCore.Qt.AlignRight)
        self.customCommand.setText('Enter Command')
        self.customCommand.setFixedWidth(150)
        self.commandButton = QtWidgets.QPushButton('Send Command')
        self.commandButton.clicked.connect(lambda: self.sendCustomCommand(self.customCommand.text() + '\r\n'))
        self.receivedMsg = QtWidgets.QLineEdit()
        self.receivedMsg.setAlignment(QtCore.Qt.AlignRight)
        self.receivedMsg.setText('LoRa response')
        self.receivedMsg.setFixedWidth(150)
        loraVLayout.addWidget(self.allSet)
        loraVLayout.addWidget(self.setParameters)
        loraVLayout.addWidget(self.testLora)
        loraVLayout.addWidget(self.customCommand)
        loraVLayout.addWidget(self.commandButton)
        loraVLayout.addWidget(self.receivedMsg)
        loraLayout.addLayout(loraOptionsLayout)
        loraLayout.addLayout(loraVLayout)
        loraLayout.addStretch()
        portOptionsText = QtWidgets.QLabel()
        portOptionsText.setText('Port Options')
        portOptionsText.setFont(self.font16)
        portOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        loraOptionsText = QtWidgets.QLabel()
        loraOptionsText.setText('LoRa Options')
        loraOptionsText.setFont(self.font16)
        loraOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        mapOptionsText = QtWidgets.QLabel()
        mapOptionsText.setText('Map Options')
        mapOptionsText.setFont(self.font16)
        mapOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        instructionsText = QtWidgets.QLabel()
        instructionsText.setText('Instructions')
        instructionsText.setFont(self.font16)
        instructionsText.setAlignment(QtCore.Qt.AlignLeft)
        keybindsText = QtWidgets.QLabel()
        keybindsText.setText('Keybinds')
        keybindsText.setFont(self.font16)
        keybindsText.setAlignment(QtCore.Qt.AlignLeft)
        self.autoPan = QtWidgets.QCheckBox('Automatically pan to rover\'s location')
        self.autoPan.setChecked(True)
        layout.addWidget(portOptionsText)
        layout.addLayout(portLayout)
        layout.addWidget(loraOptionsText)
        layout.addLayout(loraLayout)
        layout.addWidget(mapOptionsText)
        layout.addWidget(self.autoPan)
        layout.addWidget(instructionsText)
        instruction = QtWidgets.QLabel('1. Change Port/LoRa settings.\n2. Select COM port.\n3. Select control mode.')
        instruction.setWordWrap(True)
        layout.addWidget(instruction)
        layout.addWidget(keybindsText)
        layout.addWidget(QtWidgets.QLabel('W: Forward'))
        layout.addWidget(QtWidgets.QLabel('A: Left'))
        layout.addWidget(QtWidgets.QLabel('S: Reverse'))
        layout.addWidget(QtWidgets.QLabel('D: Right'))
        layout.addStretch()
        SHTab.setLayout(layout)
        return SHTab
    # message box template
    def msgBox(self, t, m, b):
        msg = QtWidgets.QMessageBox()
        msg.setText(m)
        msg.setWindowTitle(t)
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
        if b != 'OK':
            msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.exec_()
    # initialize LoRa connection
    def initLora(self):
        self.readState = 0
        self.sendCommand(self.loraCommands[0])
        line = self.readLine()
        self.readState = 1
        if line != 'Invalid response' and line != 'Timeout':
            return 1
        elif line == 'Invalid response':
            self.msgBox('ERROR', 'Invalid response', 'ERROR')
        return 0
    # request information about LoRa
    def loraTest(self):
        text = ''
        if self.connected == True:
            if self.initLora() == 1:
                self.readState = 0
                for i in range(len(self.loraCommands) - 1):
                    self.sendCommand(self.loraCommands[i + 1])
                    line = self.readLine()
                    if line == 'Timeout':
                        return
                    elif line == 'Invalid response':
                        self.msgBox('ERROR', 'Invalid response', 'ERROR')
                    else:
                        text += line + '\n'
                self.readState = 1
                self.msgBox('LoRa Info', text, 'OK')
        else:
            self.msgBox('ERROR', 'ERROR: Serial connection not established.', 'ERROR')
    # set LoRa parameters
    def setParams(self, o):
        result = self.sendCustomCommand('AT+PARAMETER=' + self.spreadingFactor.text() + \
            ',' + self.bandwidth.text() + ',' + self.codingRate.text() + ',' + self.preamble.text() + '\r\n')
        if result and o:
            self.msgBox(' ', 'Parameters set', 'OK')
        elif result and not o: 
            return 1
        else:  
            return 0
    # set all LoRa settings
    def setAll(self):
        result = self.setParams(0)
        if result: result = self.sendCustomCommand('AT+IPR=' + self.uart.text() + '\r\n')
        else: return
        if result: result = self.sendCustomCommand('AT+BAND=' + self.band.text() + '\r\n')
        else: return
        if result: result = self.sendCustomCommand('AT+NETWORKID=' + self.networkID.text() + '\r\n')
        else: return
        if result: result = self.sendCustomCommand('AT+ADDRESS=' + self.gsAddress.text() + '\r\n')
        else: return
        self.msgBox(' ', 'Everything set', 'OK')
    # send command to LoRa 
    def sendCommand(self, c):
        self.writeQueue.put(c)
        self.sent.setText(c)
    # send custom command to LoRa
    def sendCustomCommand(self, c):
        if self.connected == False:
            self.msgBox('ERROR', 'ERROR: Serial connection not established.', 'ERROR')
            return
        self.readState = 0
        self.sendCommand(c)
        line = self.readLine()
        self.readState = 1
        if line != 'Invalid response' and line != 'Timeout':
            self.receivedMsg.setText(line)
            return 1
        else:
            self.receivedMsg.setText('No response')
            return 0
    # Handle input from user
    def buttonPressed(self, button):
        if button == 't':
            if self.travelState == 1:
                data = 'Lat: ' + self.destLat.text() + ' Long: ' + self.destLong.text()
                self.sendCommand('AT+SEND=' + self.roverAddress.text() + ',' + str(len(data)) + ',' + data + '\r\n')
                self.travelState = 0
                self.destinationFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + 'Lat: ' + \
                    self.destLat.text() + ' Long: ' + self.destLong.text() + '\n') 
                self.travel.setText('Cancel')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon4);')
            else:
                data = 'Cancel'
                self.sendCommand('AT+SEND=' + self.roverAddress.text() + ',' + str(len(data)) + ',' + data + '\r\n')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
                self.destMarker.unbindTooltip()
                self.destLat.setText('')
                self.destLong.setText('')
                self.travel.setDisabled(True)
                self.travelState = 1
                self.travel.setText('Travel')
        else:
            self.sendCommand('AT+SEND=' + self.roverAddress.text() + ',' + str(len(button)) + ',' + button + '\r\n')
    # switch map style
    def mapToggle(self):
        if self.currentMap: self.currentMap = 0
        else: self.currentMap = 1
        L.tileLayer(self.maps[self.currentMap], {'maxNativeZoom': 19, 'maxZoom': 25}).addTo(self.map)
    # pan map to location
    def panTo(self, location):
        if(location == 's'): self.map.panTo(self.originalCoordinate)
        else: self.map.panTo(self.coordinate)
    # listen for keypresses
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_U:
            self.update('u')
        if event.key() == QtCore.Qt.Key_H:
            self.update('h')
        if event.key() == QtCore.Qt.Key_J:
            self.update('j')
        if event.key() == QtCore.Qt.Key_K:
            self.update('k')
        if self.controlList.currentIndex() == 2: 
            if event.key() == QtCore.Qt.Key_W:
                self.buttonPressed('w')
            if event.key() == QtCore.Qt.Key_A:
                self.buttonPressed('a')
            if event.key() == QtCore.Qt.Key_S:
                self.buttonPressed('s')
            if event.key() == QtCore.Qt.Key_D:
                self.buttonPressed('d')
    # update map/logs
    def update(self, d):
        self.updateGPS(d)
        self.log()
    # update map
    def updateGPS(self, d):
        if d == 'u': self.coordinate[0] += 0.0002
        elif d == 'h': self.coordinate[1] -= 0.0002
        elif d == 'j': self.coordinate[0] -= 0.0002
        elif d == 'k': self.coordinate[1] += 0.0002
        lat = round(self.coordinate[0], 6)
        long = round(self.coordinate[1], 6)
        self.latText.setText('Lat: ' + str(lat))
        self.longText.setText('Long: ' + str(long))
        self.distance = round(self.getDistance(self.originalCoordinate, self.coordinate), 3)
        self.distanceText.setText('Distance: ' + str(self.distance) + ' m')
        self.marker2.setLatLng(self.coordinate)
        self.map.runJavaScript('polyline.addLatLng([' + str(self.coordinate[0]) + ',' + str(self.coordinate[1]) + '])')
        if(self.autoPan.isChecked()): self.map.panTo(self.coordinate)
    # distance calculation
    def getDistance(self, start, current):
        p = pi / 180
        a = 0.5 - cos((current[0] - start[0]) * p) / 2 + cos(start[0] * p) * cos(current[0] * p) * (1 - cos((current[1] - start[1]) * p)) / 2
        return 12742 * asin(sqrt(a)) * 1000
    # log information to file
    def log(self):
        self.coordinatesFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']   Lat: ' + str(self.coordinate[0]) \
            + '  Long: ' + str(self.coordinate[1]) + '\n')
        self.telemetryFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + self.batteryText.text() + '  ' \
            + self.speedText.text() + '  ' + self.distanceText.text() + '\n')
    # handle port switches
    def switchPort(self):
        p = self.ports[self.portList.currentIndex()]
        if p == ' ':
            if self.connected == True: 
                self.connected = False
                self.controlList.setDisabled(True)
                self.controlList.setCurrentIndex(0)
                self.hideControls(True, 'all')
                self.travel.setText('Travel')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
                self.destMarker.unbindTooltip()
                self.destLat.setText('')
                self.destLong.setText('')
                self.address.setText('Rover\'s Address: NA')
                self.data.setText('Data: NA')
                self.signal.setText('Signal: NA')
                self.travel.setDisabled(True)
                if self.travelState == 0:
                    self.sendCommand('Cancel \r\n')
                self.travelState = 1
                self.readThread.join()
                self.writeThread.join()
                window.resize(700,720)
                self.currentPort = p
                self.serialPort.close()
                return
            else: 
                self.currentPort = p
                return 
        elif p == self.currentPort:
            return
        else: 
            self.currentPort = p
        if self.connected == True:
            self.connected = False
            self.controlList.setCurrentIndex(0)
            self.hideControls(True, 'all')
            self.travel.setText('Travel')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
            self.destMarker.unbindTooltip()
            self.destLat.setText('')
            self.destLong.setText('')
            self.address.setText('Rover\'s Address: NA')
            self.data.setText('Data: NA')
            self.signal.setText('Signal: NA')
            self.travel.setDisabled(True)
            if self.travelState == 0:
                self.sendCommand('Cancel \r\n')
            self.travelState = 1
            self.readThread.join()
            self.writeThread.join()
            window.resize(700,720)
            self.currentPort = p
            self.serialPort.close()
        self.serialPort = serial.Serial(port = p, baudrate = int(self.baudrateText.text()), bytesize = int(self.bytesizeText.text()), \
            timeout = int(self.timeoutText.text()), stopbits = serial.STOPBITS_ONE)
        self.readThread = threading.Thread(target = self.read, args = [self.serialPort], daemon = True)
        self.writeThread = threading.Thread(target = self.write, args = [self.serialPort], daemon = True)
        self.controlList.setDisabled(False)
        self.connected = True
        self.readThread.start()
        self.writeThread.start()
        if self.initLora() == 0:
            self.portList.setCurrentIndex(0)
            self.switchPort()
        else:
            self.setAll()
    # handle control switches
    def switchControl(self):
        if self.controlList.currentIndex() == 0: 
            self.hideControls(True, 'all')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
            self.travel.setText('Travel')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
            self.destMarker.unbindTooltip()
            self.destLat.setText('')
            self.destLong.setText('')
            self.travel.setDisabled(True)
            if self.travelState == 0:
                self.sendCommand('Cancel \r\n')
            self.travelState = 1
            window.resize(700,720)
        elif self.controlList.currentIndex() == 1: 
            self.hideControls(True, 'manual')
            self.hideControls(False, 'blind')
            window.resize(700,750)
        else:
            self.hideControls(True, 'blind')
            self.hideControls(False, 'manual')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
            self.travel.setText('Travel')
            self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
            self.destMarker.unbindTooltip()
            self.destLat.setText('')
            self.destLong.setText('')
            self.travel.setDisabled(True)
            if self.travelState == 0:
                self.sendCommand('Cancel \r\n')
            self.travelState = 1
            window.resize(700,920)
    # show/hide controls 
    def hideControls(self, h, o):
        if o == 'all':
            self.forward.setHidden(h)
            self.left.setHidden(h)
            self.reverse.setHidden(h)
            self.right.setHidden(h)
            self.destLat.setHidden(h)
            self.destLong.setHidden(h)
            self.travel.setHidden(h)
        if o == 'manual':
            self.forward.setHidden(h)
            self.left.setHidden(h)
            self.reverse.setHidden(h)
            self.right.setHidden(h)
        else: 
            self.destLat.setHidden(h)
            self.destLong.setHidden(h)
            self.travel.setHidden(h)
    # update rover destination 
    def setDest(self, dest):
        if self.travelState == 0:
            return
        if self.controlList.currentIndex() == 1:
            self.destLat.setText(str(dest['lat']))
            self.destLong.setText(str(dest['lng']))
            self.destMarker.setLatLng([dest['lat'], dest['lng']])
            self.travel.setDisabled(False)
            self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(1)')
            self.destMarker.bindTooltip('Destination')
    # write to LoRa's serial port
    def write(self, ser):
        while self.connected:
            if not self.writeQueue.empty():
                ser.write(self.writeQueue.get().encode('Ascii'))
    # read data transmitted from rover
    def read(self, ser):
        while self.connected:
            if(ser.in_waiting > 0 and self.readState == 1):
                line = ser.readline().decode('Ascii')
                print(line)
                self.receivedFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + line + '\n') 
                if line[:5] == '+RCV=':
                    line = line[5:]
                    self.message = line.split(',')
                    self.address.setText('Rover\'s Address: ' + self.message[0])
                    self.data.setText('Data: ' + self.message[2])
                    self.signal.setText('Signal: ' + self.message[3] + ' dBm')
    # wait/read LoRa response
    def readLine(self):
        start = time.time()
        while self.connected and (time.time() - start) < 5.0:
            if(self.serialPort.in_waiting > 0):
                line = self.serialPort.readline().decode('Ascii')
                print(line)
                if line[0] == '+':
                    return line[1:]
                return 'Invalid response'
        self.msgBox('ERROR', 'ERROR: No response from LoRa after 5 seconds.', 'ERROR')
        return 'Timeout'
    # handle exit request
    def closeEvent(self, event):
        if self.connected == True:
            self.connected = False
            self.portList.setCurrentIndex(0)
        msg = 'Are you sure you want to exit the program?'
        reply = QtWidgets.QMessageBox.question(self, 'Exit', msg, QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.coordinatesFile.close()
            self.telemetryFile.close()
            self.destinationFile.close()
            self.receivedFile.close()
            self.serialPort.close()
            event.accept()
        else:
            event.ignore()
        
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.resize(700,720)
    window.show()
    sys.exit(app.exec_())