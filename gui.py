from PyQt5 import QtCore, QtGui, QtWidgets
from qt_material import apply_stylesheet
from pyqtlet import L, MapWidget
from math import cos, asin, sqrt, pi
from datetime import datetime
import os
import time
import threading
import queue
import serial
import serial.tools.list_ports

class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # Connection and Write threads
        self.connectionThread = threading.Thread()
        self.writeThread = threading.Thread()
        self.writeBuf = queue.Queue() # write buffer
        self.commandBuf = queue.Queue() # command buffer
        self.lastMsg = ''; # last msg sent to rover
        self.connectionState = 'CLOSED'
        self.readState = 0
        self.closeFlag = 0
        self.seqNum = 0
        self.ackNum = 0
        self.loraCommands = ['AT\r\n', 'AT+VER?\r\n', 'AT+UID?\r\n', 'AT+BAND?\r\n', 'AT+NETWORKID?\r\n',
            'AT+ADDRESS?\r\n', 'AT+PARAMETER?\r\n', 'AT+IPR?\r\n']
        self.message = ['NA','NA','NA','NA','NA'] # LoRa message structure
        self.currentPort = ' ' 
        self.ports = [' '] # list of serial ports
        self.portsFullName = [' '] # verbose list of serial ports
        self.connected = False # serial connection state
        self.serialPort = serial.Serial()
        self.setWindowIcon(QtGui.QIcon('images/icon.png'))
        self.setWindowTitle('Ground Station')
        # create logs folder/files
        if(not os.path.exists('logs')): os.makedirs('logs')
        self.coordinatesFile = open('logs/coordinates.txt', 'a')  
        self.telemetryFile = open('logs/telemetry.txt', 'a')  
        self.sentFile = open('logs/sent.txt', 'a')  
        self.receivedFile = open('logs/received.txt', 'a')  
        # Layouts
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.CMTabUI(), 'Controls/Map')
        tabs.addTab(self.CTabUI(), 'Communication')
        tabs.addTab(self.STabUI(), 'Settings/Debug')
        self.layout.addWidget(tabs)

    def CMTabUI(self):
        self.coordinate = [0, 0]
        self.originalCoordinate = [0, 0]
        self.destinationCoordinate = [0, 0]
        CMTab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        self.layout.setSpacing(0)
        self.initLayout = QtWidgets.QGridLayout()
        self.initLayout.setVerticalSpacing(10)
        self.initLayout.setHorizontalSpacing(10)
        self.manualLayout = QtWidgets.QGridLayout()
        self.manualLayout.setVerticalSpacing(10)
        self.manualLayout.setHorizontalSpacing(10)
        self.manualLayout2 = QtWidgets.QGridLayout()
        self.manualLayout2.setVerticalSpacing(10)
        self.manualLayout2.setHorizontalSpacing(10)
        self.blindDriveLayout = QtWidgets.QGridLayout()
        self.blindDriveLayout.setVerticalSpacing(10)
        self.blindDriveLayout.setHorizontalSpacing(10)
        self.telemetryLayout = QtWidgets.QGridLayout()
        self.telemetryLayout.setVerticalSpacing(10)
        self.telemetryLayout.setHorizontalSpacing(10)
        self.portText = QtWidgets.QLabel()
        self.portText.setText('Port')
        self.portText.setProperty('class', 'header')
        self.portText.setAlignment(QtCore.Qt.AlignCenter)
        self.portList = QtWidgets.QComboBox()
        self.portList.addItems(self.portsFullName)
        self.portList.activated.connect(self.switchPort)
        self.scanPorts() # scan serial ports
        self.controlText = QtWidgets.QLabel()
        self.controlText.setText('Control Mode')
        self.controlText.setProperty('class', 'header')
        self.controlText.setAlignment(QtCore.Qt.AlignCenter)
        self.controlList = QtWidgets.QComboBox()
        self.controlList.addItems(['', 'Blind Drive', 'Manual 1', 'Manual 2'])
        self.controlList.activated.connect(self.switchControl)
        self.controlList.setDisabled(True)
        self.pointXText = QtWidgets.QLabel()
        self.pointXText.setText('X')
        self.pointXText.setProperty('class', 'header')
        self.pointXText.setAlignment(QtCore.Qt.AlignCenter)
        self.pointX = QtWidgets.QLineEdit()
        self.pointX.setMaxLength(10)
        self.pointX.setProperty('class', 'font_14')
        self.pointX.setAlignment(QtCore.Qt.AlignCenter)
        self.pointX.setText('0')
        self.pointYText = QtWidgets.QLabel()
        self.pointYText.setText('Y')
        self.pointYText.setProperty('class', 'header')
        self.pointYText.setAlignment(QtCore.Qt.AlignCenter)
        self.pointY = QtWidgets.QLineEdit()
        self.pointY.setMaxLength(10)
        self.pointY.setProperty('class', 'font_14')
        self.pointY.setAlignment(QtCore.Qt.AlignCenter)
        self.pointY.setText('0')
        self.pointZText = QtWidgets.QLabel()
        self.pointZText.setText('Z')
        self.pointZText.setProperty('class', 'header')
        self.pointZText.setAlignment(QtCore.Qt.AlignCenter)
        self.pointZ = QtWidgets.QLineEdit()
        self.pointZ.setMaxLength(10)
        self.pointZ.setProperty('class', 'font_14')
        self.pointZ.setAlignment(QtCore.Qt.AlignCenter)
        self.pointZ.setText('0')
        self.startText = QtWidgets.QLabel()
        self.startText.setText('Start')
        self.startText.setProperty('class', 'header')
        self.startText.setAlignment(QtCore.Qt.AlignCenter)
        self.cancelText = QtWidgets.QLabel()
        self.cancelText.setText('Cancel')
        self.cancelText.setProperty('class', 'header')
        self.cancelText.setAlignment(QtCore.Qt.AlignCenter)
        self.shutdownText = QtWidgets.QLabel()
        self.shutdownText.setText('Shutdown')
        self.shutdownText.setProperty('class', 'header')
        self.shutdownText.setAlignment(QtCore.Qt.AlignCenter)
        self.rcPreemptText = QtWidgets.QLabel()
        self.rcPreemptText.setText('RC Preempt')
        self.rcPreemptText.setProperty('class', 'header')
        self.rcPreemptText.setAlignment(QtCore.Qt.AlignCenter)
        self.posePreemptText = QtWidgets.QLabel()
        self.posePreemptText.setText('Pose Preempt')
        self.posePreemptText.setProperty('class', 'header')
        self.posePreemptText.setAlignment(QtCore.Qt.AlignCenter)
        self.startList = QtWidgets.QComboBox()
        self.startList.addItems(['True', 'False'])
        self.startList.setCurrentIndex(1)
        self.cancelList = QtWidgets.QComboBox()
        self.cancelList.addItems(['True', 'False'])
        self.cancelList.setCurrentIndex(1)
        self.shutdownList = QtWidgets.QComboBox()
        self.shutdownList.addItems(['True', 'False'])
        self.shutdownList.setCurrentIndex(1)
        self.rcPreemptList = QtWidgets.QComboBox()
        self.rcPreemptList.addItems(['True', 'False'])
        self.rcPreemptList.setCurrentIndex(1)
        self.posePreemptList = QtWidgets.QComboBox()
        self.posePreemptList.addItems(['True', 'False'])
        self.posePreemptList.setCurrentIndex(1)
        self.manualButton = QtWidgets.QPushButton('Send Command')
        self.manualButton.clicked.connect(lambda: self.buttonPressed('2')) 
        self.forwardText = QtWidgets.QLabel()
        self.forwardText.setText('Forward')
        self.forwardText.setProperty('class', 'header')
        self.forwardText.setAlignment(QtCore.Qt.AlignCenter)
        self.forward = QtWidgets.QLineEdit()
        self.forward.setMaxLength(10)
        self.forward.setProperty('class', 'font_14')
        self.forward.setAlignment(QtCore.Qt.AlignCenter)
        self.forward.setText('0')
        self.leftText = QtWidgets.QLabel()
        self.leftText.setText('Left')
        self.leftText.setProperty('class', 'header')
        self.leftText.setAlignment(QtCore.Qt.AlignCenter)
        self.left = QtWidgets.QLineEdit()
        self.left.setMaxLength(10)
        self.left.setProperty('class', 'font_14')
        self.left.setAlignment(QtCore.Qt.AlignCenter)
        self.left.setText('0')
        self.reverseText = QtWidgets.QLabel()
        self.reverseText.setText('Reverse')
        self.reverseText.setProperty('class', 'header')
        self.reverseText.setAlignment(QtCore.Qt.AlignCenter)
        self.reverse = QtWidgets.QLineEdit()
        self.reverse.setMaxLength(10)
        self.reverse.setProperty('class', 'font_14')
        self.reverse.setAlignment(QtCore.Qt.AlignCenter)
        self.reverse.setText('0')
        self.rightText = QtWidgets.QLabel()
        self.rightText.setText('Right')
        self.rightText.setProperty('class', 'header')
        self.rightText.setAlignment(QtCore.Qt.AlignCenter)
        self.right = QtWidgets.QLineEdit()
        self.right.setMaxLength(10)
        self.right.setProperty('class', 'font_14')
        self.right.setAlignment(QtCore.Qt.AlignCenter)
        self.right.setText('0')
        self.manualButton2 = QtWidgets.QPushButton('Send Command')
        self.manualButton2.clicked.connect(lambda: self.buttonPressed('3')) 
        self.destLat = QtWidgets.QLineEdit()
        self.destLat.setMaxLength(10)
        self.destLat.setAlignment(QtCore.Qt.AlignCenter)
        self.destLat.setProperty('class', 'font_14')
        self.destLong = QtWidgets.QLineEdit()
        self.destLong.setMaxLength(11)
        self.destLong.setAlignment(QtCore.Qt.AlignCenter)
        self.destLong.setProperty('class', 'font_14')
        self.travelState = 1
        self.travel = QtWidgets.QPushButton('Travel')
        self.travel.clicked.connect(lambda: self.buttonPressed('1')) 
        self.travel.setDisabled(True)
        self.stateText = QtWidgets.QLabel()
        self.stateText.setText('State: NA')
        self.stateText.setProperty('class', 'font_14')
        self.stateText.setAlignment(QtCore.Qt.AlignCenter)
        self.posXText = QtWidgets.QLabel()
        self.posXText.setText('Pos X: NA')
        self.posXText.setProperty('class', 'font_14')
        self.posXText.setAlignment(QtCore.Qt.AlignCenter)
        self.posYText = QtWidgets.QLabel()
        self.posYText.setText('Pos Y: NA')
        self.posYText.setProperty('class', 'font_14')
        self.posYText.setAlignment(QtCore.Qt.AlignCenter)
        self.posZText = QtWidgets.QLabel()
        self.posZText.setText('Pos Z: NA')
        self.posZText.setProperty('class', 'font_14')
        self.posZText.setAlignment(QtCore.Qt.AlignCenter)
        self.altText = QtWidgets.QLabel()
        self.altText.setText('Altitude: NA')
        self.altText.setProperty('class', 'font_14')
        self.altText.setAlignment(QtCore.Qt.AlignCenter)
        self.latText = QtWidgets.QLabel()
        self.latText.setText('Lat: ' + str(self.originalCoordinate[0]))
        self.latText.setProperty('class', 'font_14')
        self.latText.setAlignment(QtCore.Qt.AlignCenter)
        self.longText = QtWidgets.QLabel()
        self.longText.setText('Long: ' + str(self.originalCoordinate[1]))
        self.longText.setProperty('class', 'font_14')
        self.longText.setAlignment(QtCore.Qt.AlignCenter)
        self.distanceText = QtWidgets.QLabel()
        self.distanceText.setText('Distance: 0 m')
        self.distanceText.setProperty('class', 'font_14')
        self.distanceText.setAlignment(QtCore.Qt.AlignCenter)
        self.starting = QtWidgets.QPushButton('Starting Location')
        self.starting.clicked.connect(lambda: self.panTo('s')) 
        self.current = QtWidgets.QPushButton('Current Location')
        self.current.clicked.connect(lambda: self.panTo('c')) 
        self.toggle = QtWidgets.QPushButton('Toggle Map Style')
        self.toggle.clicked.connect(lambda: self.mapToggle()) 
        self.initLayout.addWidget(self.portText, 0, 0)
        self.initLayout.addWidget(self.controlText, 0, 2)
        self.initLayout.addWidget(self.portList, 1, 0)
        self.initLayout.addWidget(self.controlList, 1, 2)
        self.manualLayout.addWidget(self.pointXText, 0, 1)
        self.manualLayout.addWidget(self.pointYText, 0, 2)
        self.manualLayout.addWidget(self.pointZText, 0, 3)
        self.manualLayout.addWidget(self.pointX, 1, 1)
        self.manualLayout.addWidget(self.pointY, 1, 2)
        self.manualLayout.addWidget(self.pointZ, 1, 3)
        self.manualLayout.addWidget(self.startText, 2, 0)
        self.manualLayout.addWidget(self.cancelText, 2, 1)
        self.manualLayout.addWidget(self.shutdownText, 2, 2)
        self.manualLayout.addWidget(self.rcPreemptText, 2, 3)
        self.manualLayout.addWidget(self.posePreemptText, 2, 4)
        self.manualLayout.addWidget(self.startList, 3, 0)
        self.manualLayout.addWidget(self.cancelList, 3, 1)
        self.manualLayout.addWidget(self.shutdownList, 3, 2)
        self.manualLayout.addWidget(self.rcPreemptList, 3, 3)
        self.manualLayout.addWidget(self.posePreemptList, 3, 4)
        self.manualLayout.addWidget(self.manualButton, 4, 2)
        self.manualLayout.setColumnStretch(0, 1)
        self.manualLayout.setColumnStretch(1, 1)
        self.manualLayout.setColumnStretch(2, 1)
        self.manualLayout.setColumnStretch(3, 1)
        self.manualLayout.setColumnStretch(4, 1)
        self.manualLayout2.addWidget(self.forwardText, 0, 1)
        self.manualLayout2.addWidget(self.forward, 1, 1)
        self.manualLayout2.addWidget(self.leftText, 2, 0)
        self.manualLayout2.addWidget(self.reverseText, 2, 1)
        self.manualLayout2.addWidget(self.rightText, 2, 2)
        self.manualLayout2.addWidget(self.left, 3, 0)
        self.manualLayout2.addWidget(self.reverse, 3, 1)
        self.manualLayout2.addWidget(self.right, 3, 2)
        self.manualLayout2.addWidget(self.manualButton2, 4, 1)
        self.manualLayout2.setColumnStretch(0, 1)
        self.manualLayout2.setColumnStretch(1, 1)
        self.manualLayout2.setColumnStretch(2, 1)
        self.blindDriveLayout.addWidget(self.destLat, 0, 0)
        self.blindDriveLayout.addWidget(self.destLong, 0, 1)
        self.blindDriveLayout.addWidget(self.travel, 0, 2)
        self.telemetryLayout.addWidget(self.stateText, 0, 0)
        self.telemetryLayout.addWidget(self.altText, 0, 2)
        self.telemetryLayout.addWidget(self.posXText, 1, 0)
        self.telemetryLayout.addWidget(self.posYText, 1, 1)
        self.telemetryLayout.addWidget(self.posZText, 1, 2)
        self.telemetryLayout.addWidget(self.latText, 2, 0)
        self.telemetryLayout.addWidget(self.longText, 2, 1)
        self.telemetryLayout.addWidget(self.distanceText, 2, 2)
        self.telemetryLayout.addWidget(self.starting, 3, 0)
        self.telemetryLayout.addWidget(self.current, 3, 1)
        self.telemetryLayout.addWidget(self.toggle, 3, 2)
        self.mapLayout = QtWidgets.QVBoxLayout()
        self.mapWidget = MapWidget()
        self.map = L.map(self.mapWidget) 
        self.map.setView(self.originalCoordinate, 18)
        self.worldMap = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
        self.darkMap = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
        self.maps = [self.darkMap, self.worldMap]
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
        self.map.clicked.connect(lambda x: self.setDest(x['latlng']))
        self.mapLayout.addWidget(self.mapWidget)
        self.hideControls(True, 'all')
        self.spaceItem = QtWidgets.QSpacerItem(150, 30, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.spaceItem2 = QtWidgets.QSpacerItem(150, 30, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        layout.addLayout(self.initLayout)
        layout.addItem(self.spaceItem)
        layout.addLayout(self.manualLayout)
        layout.addLayout(self.manualLayout2)
        layout.addLayout(self.blindDriveLayout)
        layout.addItem(self.spaceItem2)
        layout.addLayout(self.telemetryLayout)
        layout.addLayout(self.mapLayout)
        layout.addStretch()
        CMTab.setLayout(layout)
        return CMTab

    def CTabUI(self):
        CTab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(5)
        self.communicationLayout = QtWidgets.QGridLayout()
        self.communicationLayout.setVerticalSpacing(50)
        self.communicationLayout.setHorizontalSpacing(30)
        self.connStatusText = QtWidgets.QLabel()
        self.connStatusText.setText('Connection Status: ')
        self.connStatusText.setProperty('class', 'header')
        self.connStatusText.setAlignment(QtCore.Qt.AlignLeft)
        self.connStatus = QtWidgets.QLabel()
        self.connStatus.setText('CLOSED')
        self.connStatus.setProperty('class', 'danger')
        self.connStatus.setAlignment(QtCore.Qt.AlignLeft)
        self.address = QtWidgets.QLabel()
        self.address.setText('Rover\'s Address: NA')
        self.address.setProperty('class', 'header')
        self.address.setAlignment(QtCore.Qt.AlignLeft)
        self.rssi = QtWidgets.QLabel()
        self.rssi.setText('RSSI: NA')
        self.rssi.setProperty('class', 'header')
        self.rssi.setAlignment(QtCore.Qt.AlignLeft)
        self.snr = QtWidgets.QLabel()
        self.snr.setText('SNR: NA')
        self.snr.setProperty('class', 'header')
        self.snr.setAlignment(QtCore.Qt.AlignLeft)
        self.sentText = QtWidgets.QLabel()
        self.sentText.setText('Sent Data:')
        self.sentText.setProperty('class', 'header')
        self.sentText.setAlignment(QtCore.Qt.AlignLeft)
        self.sent = QtWidgets.QLabel()
        self.sent.setText('NA')
        self.sent.setProperty('class', 'font_14')
        self.sent.setAlignment(QtCore.Qt.AlignLeft)
        self.sent.setWordWrap(True)
        self.receivedText = QtWidgets.QLabel()
        self.receivedText.setText('Received Data:')
        self.receivedText.setProperty('class', 'header')
        self.receivedText.setAlignment(QtCore.Qt.AlignLeft)
        self.received = QtWidgets.QLabel()
        self.received.setText('NA')
        self.received.setProperty('class', 'font_14')
        self.received.setAlignment(QtCore.Qt.AlignLeft)
        self.received.setWordWrap(True)
        self.sentStatus = QtWidgets.QLabel()
        self.sentStatus.setText('NA')
        self.sentStatus.setProperty('class', 'header')
        self.sentStatus.setAlignment(QtCore.Qt.AlignLeft)
        self.closeConnection = QtWidgets.QPushButton('Close Connection')
        self.closeConnection.clicked.connect(lambda: self.close()) 
        self.closeConnection.setDisabled(True)
        self.communicationLayout.addWidget(self.address, 0, 0)
        self.communicationLayout.addWidget(self.rssi, 0, 1)
        self.communicationLayout.addWidget(self.snr, 0, 2)
        self.communicationLayout.addWidget(self.connStatusText, 1, 0)
        self.communicationLayout.addWidget(self.connStatus, 1, 1, 1, 2)
        self.communicationLayout.addWidget(self.sentText, 2, 0)
        self.communicationLayout.addWidget(self.sent, 2, 1, 1, 2)
        self.communicationLayout.addWidget(self.sentStatus, 3, 0)
        self.communicationLayout.addWidget(self.receivedText, 4, 0)
        self.communicationLayout.addWidget(self.received, 4, 1, 1, 2)
        self.communicationLayout.addWidget(self.closeConnection, 5, 1)
        layout.addLayout(self.communicationLayout)
        layout.addStretch()
        CTab.setLayout(layout)
        return CTab

    def STabUI(self):
        SHTab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(5)
        portLayout = QtWidgets.QFormLayout()
        loraLayout = QtWidgets.QHBoxLayout()
        loraLayout.setSpacing(10)
        loraVLayout = QtWidgets.QVBoxLayout()
        loraOptionsLayout = QtWidgets.QFormLayout()
        self.scanButton = QtWidgets.QPushButton('Scan Ports')
        self.scanButton.clicked.connect(lambda: self.scanPorts()) 
        self.baudrateText = QtWidgets.QLineEdit()
        self.baudrateText.setAlignment(QtCore.Qt.AlignCenter)
        self.baudrateText.setText('115200')
        self.baudrateText.setFixedWidth(70)
        self.baudrateText.setProperty('class', 'font_12')
        self.bytesizeText = QtWidgets.QLineEdit()
        self.bytesizeText.setAlignment(QtCore.Qt.AlignCenter)
        self.bytesizeText.setText('8')
        self.bytesizeText.setFixedWidth(70)
        self.bytesizeText.setProperty('class', 'font_12')
        self.timeoutText = QtWidgets.QLineEdit()
        self.timeoutText.setAlignment(QtCore.Qt.AlignCenter)
        self.timeoutText.setText('2')
        self.timeoutText.setFixedWidth(70)
        self.timeoutText.setProperty('class', 'font_12')
        self.spreadingFactor = QtWidgets.QLineEdit()
        self.spreadingFactor.setAlignment(QtCore.Qt.AlignCenter)
        self.spreadingFactor.setText('12')
        self.spreadingFactor.setFixedWidth(70)
        self.spreadingFactor.setProperty('class', 'font_12')
        self.bandwidth = QtWidgets.QLineEdit()
        self.bandwidth.setAlignment(QtCore.Qt.AlignCenter)
        self.bandwidth.setText('7')
        self.bandwidth.setFixedWidth(70)
        self.bandwidth.setProperty('class', 'font_12')
        self.codingRate = QtWidgets.QLineEdit()
        self.codingRate.setAlignment(QtCore.Qt.AlignCenter)
        self.codingRate.setText('2')
        self.codingRate.setFixedWidth(70)
        self.codingRate.setProperty('class', 'font_12')
        self.preamble = QtWidgets.QLineEdit()
        self.preamble.setAlignment(QtCore.Qt.AlignCenter)
        self.preamble.setText('5')
        self.preamble.setFixedWidth(70)
        self.preamble.setProperty('class', 'font_12')
        self.gsAddress = QtWidgets.QLineEdit()
        self.gsAddress.setAlignment(QtCore.Qt.AlignCenter)
        self.gsAddress.setText('101')
        self.gsAddress.setFixedWidth(70)
        self.gsAddress.setProperty('class', 'font_12')
        self.roverAddress = QtWidgets.QLineEdit()
        self.roverAddress.setAlignment(QtCore.Qt.AlignCenter)
        self.roverAddress.setText('102')
        self.roverAddress.setFixedWidth(70)
        self.roverAddress.setProperty('class', 'font_12')
        self.networkID = QtWidgets.QLineEdit()
        self.networkID.setAlignment(QtCore.Qt.AlignCenter)
        self.networkID.setText('5')
        self.networkID.setFixedWidth(70)
        self.networkID.setProperty('class', 'font_12')
        self.band = QtWidgets.QLineEdit()
        self.band.setAlignment(QtCore.Qt.AlignCenter)
        self.band.setText('915000000')
        self.band.setFixedWidth(70)
        self.band.setProperty('class', 'font_12')
        self.uart = QtWidgets.QLineEdit()
        self.uart.setAlignment(QtCore.Qt.AlignCenter)
        self.uart.setText('115200')
        self.uart.setFixedWidth(70)
        self.uart.setProperty('class', 'font_12')
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
        loraOptionsLayout.addRow('Baudrate:', self.uart)
        self.allSet = QtWidgets.QPushButton('Set All')
        self.allSet.clicked.connect(lambda: self.setAll()) 
        self.setParameters = QtWidgets.QPushButton('Set Parameters')
        self.setParameters.clicked.connect(lambda: self.setParams(1)) 
        self.testLora = QtWidgets.QPushButton('LoRa Info')
        self.testLora.clicked.connect(lambda: self.loraTest()) 
        self.customCommand = QtWidgets.QLineEdit()
        self.customCommand.setAlignment(QtCore.Qt.AlignCenter)
        self.customCommand.setText('Enter Command')
        self.customCommand.setFixedWidth(150)
        self.commandButton = QtWidgets.QPushButton('Send Command')
        self.commandButton.clicked.connect(lambda: self.sendCustomCommand(self.customCommand.text() + '\r\n'))
        self.receivedMsg = QtWidgets.QLineEdit()
        self.receivedMsg.setAlignment(QtCore.Qt.AlignCenter)
        self.receivedMsg.setText('LoRa response')
        self.receivedMsg.setFixedWidth(150)
        loraVLayout.addWidget(self.scanButton)
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
        portOptionsText.setText('Serial Port Options')
        portOptionsText.setProperty('class', 'header')
        portOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        loraOptionsText = QtWidgets.QLabel()
        loraOptionsText.setText('LoRa Options')
        loraOptionsText.setProperty('class', 'header')
        loraOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        mapOptionsText = QtWidgets.QLabel()
        mapOptionsText.setText('Map Options')
        mapOptionsText.setProperty('class', 'header')
        mapOptionsText.setAlignment(QtCore.Qt.AlignLeft)
        self.autoPan = QtWidgets.QCheckBox('Automatically pan to rover\'s location')
        self.autoPan.setChecked(True)
        layout.addWidget(portOptionsText)
        layout.addLayout(portLayout)
        layout.addWidget(loraOptionsText)
        layout.addLayout(loraLayout)
        layout.addWidget(mapOptionsText)
        layout.addWidget(self.autoPan)
        layout.addStretch()
        SHTab.setLayout(layout)
        return SHTab
    # message box template
    def msgBox(self, t, m, b):
        msg = QtWidgets.QMessageBox(self)
        msg.setText(m)
        msg.setWindowTitle(t)
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
        if b != 'OK':
            msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.exec_()
    # scan serial ports
    def scanPorts(self):
        self.portList.clear()
        self.currentPort = ' ' 
        self.ports = [' '] # list of serial ports
        self.portsFullName = [' '] # verbose list of serial ports
        for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
            self.ports.append(port)
            self.portsFullName.append(port + ' - ' + hwid)
        self.portList.addItems(self.portsFullName)
    # initialize LoRa connection
    def initLora(self):
        self.sendCommand(self.loraCommands[0])
        line = self.readLora()
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
                for i in range(len(self.loraCommands) - 1):
                    self.sendCommand(self.loraCommands[i + 1])
                    line = self.readLora()
                    if line == 'Timeout':
                        return
                    elif line == 'Invalid response':
                        self.msgBox('ERROR', 'Invalid response', 'ERROR')
                    else:
                        text += line + '\n'
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
        print(c)
        self.writeBuf.put(c)
    # send custom command to LoRa
    def sendCustomCommand(self, c):
        if self.connected == False:
            self.msgBox('ERROR', 'ERROR: Serial connection not established.', 'ERROR')
            return
        self.sendCommand(c)
        line = self.readLora()
        if line != 'Invalid response' and line != 'Timeout':
            self.receivedMsg.setText(line)
            return 1
        else:
            self.receivedMsg.setText('No response')
            return 0
    # Handle input from user
    def buttonPressed(self, button):
        if button == '1':
            if self.travelState == 1:
                data = '[Lat:' + self.destLat.text() + '][Long:' + self.destLong.text() + ']'
                self.createTx(data, 1)
                self.travelState = 0
                self.sentFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + 'Lat: ' + \
                    self.destLat.text() + ' Long: ' + self.destLong.text() + '\n') 
                self.travel.setText('Cancel')
                self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon4);')
            else:
                self.resetM()
        elif button == '2':
            data = self.pointX.text() + ' ' + self.pointY.text() + ' ' + self.pointZ.text() + ' ' + \
                self.startList.currentText() + ' ' + self.cancelList.currentText() + ' ' + self.shutdownList.currentText() + \
                ' ' + self.rcPreemptList.currentText() +  ' ' + self.posePreemptList.currentText()
            self.createTx(data, 2)
            self.sentFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + '] X = ' + self.pointX.text() + \
                ' Y = ' + self.pointY.text() + ' Z = ' + self.pointZ.text() + ' Start = ' + self.startList.currentText() + \
                ' Cancel = ' + self.cancelList.currentText() + ' Shutdown = ' + self.shutdownList.currentText() + \
                ' RC Preempt = ' + self.rcPreemptList.currentText() +  ' Pose Preempt = ' + self.posePreemptList.currentText())
        elif button == '3':
            data = self.forward.text() + ' ' + self.reverse.text() + ' ' + self.left.text() + ' ' + self.right.text()
            self.createTx(data, 3)
            self.sentFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + '] Forward: ' + self.forward.text() + \
                ' Reverse: ' + self.reverse.text() + ' Left: ' + self.left.text() + ' Right: ' + self.right.text())
        else:
            self.createTx(button, 4)
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
    #def keyPressEvent(self, event):
    # update map/logs
    def update(self, i):
        self.updateGPS(i)
        self.log()
    # update map
    def updateGPS(self, i):
        lat = round(self.coordinate[0], 6)
        long = round(self.coordinate[1], 6)
        self.latText.setText('Lat: ' + str(lat))
        self.longText.setText('Long: ' + str(long))
        self.distance = round(self.getDistance(self.originalCoordinate, self.coordinate), 3)
        self.distanceText.setText('Distance: ' + str(self.distance) + ' m')
        self.marker2.setLatLng(self.coordinate)
        if i: 
           self.map.runJavaScript('polyline.addLatLng([' + str(self.coordinate[0]) + ',' + str(self.coordinate[1]) + '])')
        else:
            self.marker.setLatLng(self.originalCoordinate)
            self.map.runJavaScript('var startCoordinate = [[' + str(self.originalCoordinate[0]) + ',' + str(self.originalCoordinate[1]) + ']]')
            self.map.runJavaScript("var polyline = L.polyline(startCoordinate, {color: '#0077ff'}).addTo(" + self.map.jsName + ");")
            self.map.runJavaScript('polyline.bindTooltip(\"Rover\'s Path\");')
        if self.autoPan.isChecked(): self.map.panTo(self.coordinate)
    # distance calculation
    def getDistance(self, start, current):
        p = pi / 180
        a = 0.5 - cos((current[0] - start[0]) * p) / 2 + cos(start[0] * p) * cos(current[0] * p) * (1 - cos((current[1] - start[1]) * p)) / 2
        return 12742 * asin(sqrt(a)) * 1000
    # log information to file
    def log(self):
        self.coordinatesFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']   Lat: ' + str(self.coordinate[0]) \
            + '  Long: ' + str(self.coordinate[1]) + '\n')
        self.telemetryFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + self.distanceText.text() + '\n')
    # reset map and communication
    def resetMC(self, p):
        self.resetM()
        self.connected = False
        self.changeState('CLOSED', 'danger')
        self.seqNum = 0;
        self.ackNum = 0;
        self.controlList.setCurrentIndex(0)
        self.address.setText('Rover\'s Address: NA')
        self.received.setText('NA')
        self.sent.setText('NA')
        self.rssi.setText('RSSI: NA')
        self.snr.setText('SNR: NA')
        self.connectionThread.join()
        self.writeThread.join()
        self.currentPort = p
        self.serialPort.close()
        self.allSet.setDisabled(False)
        self.setParameters.setDisabled(False)
        self.testLora.setDisabled(False)
        self.commandButton.setDisabled(False)
    # reset map
    def resetM(self):
        data = 'Cancel'
        #self.sendCommand('AT+SEND=' + self.roverAddress.text() + ',' + str(len(data)) + ',' + data + '\r\n')
        self.map.runJavaScript(f'{self.destMarker.jsName}.setOpacity(0)')
        self.map.runJavaScript(f'{self.destMarker.jsName}.setIcon(markerIcon3);')
        self.destMarker.unbindTooltip()
        self.destLat.setText('')
        self.destLong.setText('')
        self.travel.setText('Travel')
        self.travel.setDisabled(True)
        self.travelState = 1
    # handle port switches
    def switchPort(self):
        p = self.ports[self.portList.currentIndex()]
        if p == ' ':
            if self.connected == True: 
                self.controlList.setDisabled(True)
                self.hideControls(True, 'all')
                self.resetMC(p)
                window.resize(700,670)
                return
            else: 
                self.currentPort = p
                return 
        elif p == self.currentPort:
            return
        else: 
            self.currentPort = p
        if self.connected == True:
            self.hideControls(True, 'all')
            self.resetMC(p)
            window.resize(700,670)
        self.serialPort = serial.Serial(port = p, baudrate = int(self.baudrateText.text()), bytesize = int(self.bytesizeText.text()), \
            timeout = int(self.timeoutText.text()), stopbits = serial.STOPBITS_ONE)
        self.connectionThread = threading.Thread(target = self.connection, args = [self.serialPort], daemon = True)
        self.writeThread = threading.Thread(target = self.write, args = [self.serialPort], daemon = True)
        self.changeState('LISTEN', 'warning')
        self.controlList.setDisabled(False)
        self.connected = True
        self.writeThread.start()
        if self.initLora() == 0:
            self.portList.setCurrentIndex(0)
            self.switchPort()
        else:
            self.setAll()
        self.connectionThread.start()
    # handle control switches
    def switchControl(self):
        if self.controlList.currentIndex() == 0: 
            self.hideControls(True, 'all')
            self.resetM()
            window.resize(700,670)
        elif self.controlList.currentIndex() == 1: 
            self.hideControls(True, 'manual 1')
            self.hideControls(True, 'manual 2')
            self.hideControls(False, 'blind')
            window.resize(700,700)
        elif self.controlList.currentIndex() == 2: 
            self.hideControls(True, 'blind')
            self.hideControls(True, 'manual 2')
            self.hideControls(False, 'manual 1')
            self.resetM()
            window.resize(700,870)
        else:
            self.hideControls(True, 'blind')
            self.hideControls(True, 'manual 1')
            self.hideControls(False, 'manual 2')
            self.resetM()
            window.resize(700,850)
    # show/hide controls 
    def hideControls(self, h, o):
        if o == 'all':
            self.pointXText.setHidden(h)
            self.pointX.setHidden(h)
            self.pointYText.setHidden(h)
            self.pointY.setHidden(h)
            self.pointZText.setHidden(h)
            self.pointZ.setHidden(h)
            self.startText.setHidden(h)
            self.cancelText.setHidden(h)
            self.shutdownText.setHidden(h)
            self.rcPreemptText.setHidden(h)
            self.posePreemptText.setHidden(h)
            self.startList.setHidden(h)
            self.cancelList.setHidden(h)
            self.shutdownList.setHidden(h)
            self.rcPreemptList.setHidden(h)
            self.posePreemptList.setHidden(h)
            self.manualButton.setHidden(h)
            self.forwardText.setHidden(h)
            self.forward.setHidden(h)
            self.leftText.setHidden(h)
            self.left.setHidden(h)
            self.reverseText.setHidden(h)
            self.reverse.setHidden(h)
            self.rightText.setHidden(h)
            self.right.setHidden(h)
            self.manualButton2.setHidden(h)
            self.destLat.setHidden(h)
            self.destLong.setHidden(h)
            self.travel.setHidden(h)
        elif o == 'manual 1':
            self.pointXText.setHidden(h)
            self.pointX.setHidden(h)
            self.pointYText.setHidden(h)
            self.pointY.setHidden(h)
            self.pointZText.setHidden(h)
            self.pointZ.setHidden(h)
            self.startText.setHidden(h)
            self.cancelText.setHidden(h)
            self.shutdownText.setHidden(h)
            self.rcPreemptText.setHidden(h)
            self.posePreemptText.setHidden(h)
            self.startList.setHidden(h)
            self.cancelList.setHidden(h)
            self.shutdownList.setHidden(h)
            self.rcPreemptList.setHidden(h)
            self.posePreemptList.setHidden(h)
            self.manualButton.setHidden(h)
        elif o == 'manual 2':
            self.forwardText.setHidden(h)
            self.forward.setHidden(h)
            self.leftText.setHidden(h)
            self.left.setHidden(h)
            self.reverseText.setHidden(h)
            self.reverse.setHidden(h)
            self.rightText.setHidden(h)
            self.right.setHidden(h)
            self.manualButton2.setHidden(h)
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
    # write to serial port
    def write(self, ser):
        while self.connected:
            if not self.writeBuf.empty():
                ser.write(self.writeBuf.get().encode('Ascii'))
    # read from serial port
    def read(self):
        if self.serialPort.in_waiting > 0:
            return self.serialPort.readline().decode()
    # create tx msg
    def createTx(self, data, option):
        if option == 1:
            msg = 'CMD BD ' + data
        elif option == 2:
            msg = 'CMD MAN1 ' + data
        elif option == 3:
            msg = 'CMD MAN2 ' + data
        self.commandBuf.put(msg)
    # command tx format
    def cmdTx(self):
        msg = self.commandBuf.get()
        msg = str(self.seqNum) + ' ' + str(self.ackNum) + ' ' + msg
        msg = 'AT+SEND=' + self.roverAddress.text() + ',' + str(len(msg)) + ',' + msg + '\r\n'
        self.lastMsg = msg
        self.sent.setText(msg)
        self.sendCommand(msg)
    # msg tx format 
    def msgTx(self, c):
        msg = str(self.seqNum) + ' ' + str(self.ackNum) + ' ' + c
        msg = 'AT+SEND=' + self.roverAddress.text() + ',' + str(len(msg)) + ',' + msg + '\r\n'
        self.lastMsg = msg
        self.sent.setText(msg)
        self.sendCommand(msg)
    # msg rx format 
    def msgRx(self, msg):
        if msg and msg[:5] == '+RCV=':
            self.received.setText(msg)
            msg = msg[5:]
            self.message = msg.split(',')
            self.address.setText('Rover\'s Address: ' + self.message[0])
            self.receivedFile.write('[' + datetime.now().strftime('%b %d %H:%M:%S') + ']  ' + self.message[2] + '\n') 
            self.rssi.setText('RSSI: ' + self.message[3] + ' dBm')
            self.snr.setText('SNR: ' + self.message[4])
            return self.message[2]
    # parse rx message
    def parseMsg(self):
        data = self.msgRx(self.read())
        if data and (' ' in data):
            data = data.split(' ')
            return data
    # change state
    def changeState(self, s, c):
        self.connectionState = s
        self.connStatus.setText(s)
        self.connStatus.setProperty('class', c)
        self.style().unpolish(self.connStatus)
        self.style().polish(self.connStatus)
    # communication state machine
    def connection(self, ser):
        while self.connected:
            if self.connectionState == 'LISTEN':
                data = self.parseMsg()
                if data and data[2] == 'SYN':
                    self.ackNum = int(data[0]) + 1
                    self.allSet.setDisabled(True)
                    self.setParameters.setDisabled(True)
                    self.testLora.setDisabled(True)
                    self.commandButton.setDisabled(True)
                    self.msgTx('SYN')
                    self.changeState('SYN-RECEIVED', 'warning')
                    self.closeConnection.setDisabled(False)
            elif self.connectionState == 'SYN-RECEIVED':
                data = self.parseMsg()
                if data and data[2] == 'ACK':
                    self.seqNum = int(data[1])
                    self.msgTx('ACK')
                    self.changeState('ESTABLISHED', 'success')
            elif self.connectionState == 'ESTABLISHED':
                data = self.parseMsg()
                if data and data[2] == 'ACK':
                    print(data)
                    print(data[0] + ' ' + str(self.ackNum))
                    if int(data[0]) != self.ackNum: 
                        self.sent.setText(self.lastMsg)
                        self.sentStatus.setText("Sequence Error: Retransmitting")
                        self.sentStatus.setProperty('class', 'danger')
                        self.sendCommand(self.lastMsg)
                    else:
                        print(len(data))
                        self.sentStatus.setText("Message sent successfully")
                        self.sentStatus.setProperty('class', 'success')
                        self.seqNum = int(data[1])
                        if len(data) > 3:
                            self.stateText.setText('State: ' + data[3])
                            self.posXText.setText('Pos X: ' + data[4])
                            self.posYText.setText('Pos Y: ' + data[5])
                            self.posZText.setText('Pos Z: ' + data[6])
                            self.coordinate = [float(data[7]), float(data[8])]
                            if self.originalCoordinate == [0,0]:
                                self.originalCoordinate = [float(data[7]), float(data[8])]
                                self.update(False)
                            else :
                                self.update(True)
                            self.altText.setText('Altitude: ' + data[9] + ' m')
                        if self.closeFlag: 
                            self.msgTx('FIN')
                            self.changeState('FIN-WAIT', 'warning')
                            self.closeFlag = 0
                        elif not self.commandBuf.empty(): 
                            self.ackNum = int(data[0]) + 1
                            self.cmdTx()
                        else:
                            self.ackNum = int(data[0]) + 1
                            self.msgTx('ACK')
            elif self.connectionState == 'FIN-WAIT':
                data = self.parseMsg()
                if data and data[2] == 'FIN':
                    self.seqNum = int(data[1])
                    self.ackNum = int(data[0]) + 1
                    self.changeState('TIME-WAIT', 'warning')
            elif self.connectionState == 'TIME-WAIT':
                self.msgTx('ACK')
                self.seqNum = 0
                self.ackNum = 0
                self.changeState('CLOSED', 'danger')
                self.scanPorts()
                time.sleep(3)
    # wait/read LoRa response
    def readLora(self):
        start = time.time()
        while self.connected and (time.time() - start) < 5.0:
            line = self.read()
            if line:
                print(line)
                if line[0] == '+':
                    return line[1:]
                return 'Invalid response'
        self.msgBox('ERROR', 'ERROR: No response from LoRa after 5 seconds.', 'ERROR')
        return 'Timeout'
    # close connnection
    def close(self):
        self.closeFlag = 1
        self.closeConnection.setDisabled(True)
    # handle exit request
    def closeEvent(self, event):
        if self.connected == True:
            self.portList.setCurrentIndex(0)
            self.switchPort()
        msg = 'Are you sure you want to exit the program?'
        reply = QtWidgets.QMessageBox.question(self, 'Exit', msg, QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            self.coordinatesFile.close()
            self.telemetryFile.close()
            self.sentFile.close()
            self.receivedFile.close()
            self.serialPort.close()
            event.accept()
        else:
            event.ignore()
        
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.resize(700,670)
    extra = {
        # Button colors
        'red': '#dc3545',
        'orange': '#ffc107',
        'green': '#17a2b8',
        # Font
        'font_family': 'Roboto',
    }
    apply_stylesheet(app, theme='themes/colors.xml', extra=extra)
    stylesheet = app.styleSheet()
    with open('themes/custom.css') as file:
        app.setStyleSheet(stylesheet + file.read().format(**os.environ))
    window.show()
    sys.exit(app.exec_())