TARGET = "/home/pi/ros_catkin_ws/devel_isolated/rviz/lib/python3/dist-packages/rviz/librviz_sip".so
OFILES = siplibrviz_sipcmodule.o siplibrviz_siprvizYamlConfigReader.o siplibrviz_siprvizYamlConfigWriter.o siplibrviz_siprvizConfigMapIterator.o siplibrviz_siprvizConfig.o siplibrviz_siprvizOgreLogging.o siplibrviz_siprvizToolManager.o siplibrviz_siprvizTool.o siplibrviz_siprvizViewManager.o siplibrviz_siprvizViewController.o siplibrviz_siprvizPanelDockWidget.o siplibrviz_siprvizBoolProperty.o siplibrviz_siprvizDisplayGroup.o siplibrviz_siprvizDisplay.o siplibrviz_siprvizProperty.o siplibrviz_siprvizVisualizationManager.o siplibrviz_siprvizVisualizationFrame.o siplibrviz_siprviz.o
HFILES = sipAPIlibrviz_sip.h 

CC = gcc
CXX = g++
LINK = g++
CPPFLAGS = -DNDEBUG -DROS_BUILD_SHARED_LIBS -DQT_NO_DEBUG -DQT_CORE_LIB -DQT_GUI_LIB -I. -I/home/pi/ros_catkin_ws/src/rviz/src -I/home/pi/ros_catkin_ws/devel_isolated/rviz/include -I/usr/include/OGRE/Overlay -I/usr/include/OGRE -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/usr/include -I/usr/include/eigen3 -I/usr/include/python3.7m -I/usr/share/qt5/mkspecs/linux-g++ -I/usr/include/arm-linux-gnueabihf/qt5/QtCore -I/usr/include/arm-linux-gnueabihf/qt5/QtGui -I/usr/include/arm-linux-gnueabihf/qt5/QtWidgets -I/usr/include/arm-linux-gnueabihf/qt5/QtPrintSupport -I/usr/include/arm-linux-gnueabihf/qt5 -I/usr/X11R6/include
CFLAGS =  -g -O2 -fdebug-prefix-map=/build/sip4-hgGQdY/sip4-4.19.14+dfsg=. -fstack-protector-strong -Wformat -Werror=format-security  -Wdate-time -D_FORTIFY_SOURCE=2 -fPIC -Wall -W -D_REENTRANT
CXXFLAGS = -std=c++14  -g -O2 -fdebug-prefix-map=/build/sip4-hgGQdY/sip4-4.19.14+dfsg=. -fstack-protector-strong -Wformat -Werror=format-security  -Wdate-time -D_FORTIFY_SOURCE=2 -fPIC -Wall -W -D_REENTRANT
LFLAGS =  -Wl,-z,relro -Wl,-rpath,"/home/pi/ros_catkin_ws/devel_isolated/rviz/lib" -shared -Wl,-rpath,/home/pi/ros_catkin_ws/devel_isolated/rviz/lib -Wl,-rpath,/usr/lib/arm-linux-gnueabihf -Wl,--version-script="/home/pi/ros_catkin_ws/devel_isolated/rviz/lib/python3/dist-packages/rviz/librviz_sip".exp
LIBS = -L/home/pi/ros_catkin_ws/devel_isolated/rviz/lib -L/usr/lib/arm-linux-gnueabihf -L/usr/X11R6/lib -lrviz -lQt5Core -lpthread -latomic -lQt5Gui -lQt5Core -lpthread -latomic -lQt5Widgets -lQt5PrintSupport -lXext -lX11 -lm -lpthread
MOC = moc
.SUFFIXES: .c .o .cpp .cc .cxx .C


.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) -o $@ $<

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) -o $@ $<

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) -o $@ $<

.C.o:
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(CPPFLAGS) -o $@ $<

all: $(TARGET)

$(OFILES): $(HFILES)

$(TARGET): $(OFILES)
	@echo '{ global: PyInit_librviz_sip; local: *; };' > "/home/pi/ros_catkin_ws/devel_isolated/rviz/lib/python3/dist-packages/rviz/librviz_sip".exp
	$(LINK) $(LFLAGS) -o $(TARGET) $(OFILES) $(LIBS)

install: $(TARGET)
	@test -d $(DESTDIR)/usr/lib/python3/dist-packages || mkdir -p $(DESTDIR)/usr/lib/python3/dist-packages
	cp -f $(TARGET) $(DESTDIR)/usr/lib/python3/dist-packages/$(TARGET)

clean:
	-rm -f $(TARGET)
	-rm -f siplibrviz_sipcmodule.o
	-rm -f siplibrviz_siprvizYamlConfigReader.o
	-rm -f siplibrviz_siprvizYamlConfigWriter.o
	-rm -f siplibrviz_siprvizConfigMapIterator.o
	-rm -f siplibrviz_siprvizConfig.o
	-rm -f siplibrviz_siprvizOgreLogging.o
	-rm -f siplibrviz_siprvizToolManager.o
	-rm -f siplibrviz_siprvizTool.o
	-rm -f siplibrviz_siprvizViewManager.o
	-rm -f siplibrviz_siprvizViewController.o
	-rm -f siplibrviz_siprvizPanelDockWidget.o
	-rm -f siplibrviz_siprvizBoolProperty.o
	-rm -f siplibrviz_siprvizDisplayGroup.o
	-rm -f siplibrviz_siprvizDisplay.o
	-rm -f siplibrviz_siprvizProperty.o
	-rm -f siplibrviz_siprvizVisualizationManager.o
	-rm -f siplibrviz_siprvizVisualizationFrame.o
	-rm -f siplibrviz_siprviz.o
	-rm -f "/home/pi/ros_catkin_ws/devel_isolated/rviz/lib/python3/dist-packages/rviz/librviz_sip".exp
