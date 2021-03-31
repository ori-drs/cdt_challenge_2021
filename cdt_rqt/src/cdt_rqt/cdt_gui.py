import os
import rospy
import rospkg
import numpy as np
import random

# Import ros messages
from std_srvs.srv import EmptyRequest, Empty, EmptyResponse
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64

# CDT messages
from cdt_msgs.msg import Graph, Frontiers, Object, ObjectList
from cdt_msgs.srv import ToggleExploration, ToggleExplorationRequest, ToggleExplorationResponse

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QSizePolicy
from python_qt_binding.QtCore import QTimer

# Thread stuff
from threading import Lock

# Import resources
import cdt_rqt.resources

class CdtGui(Plugin):
    def __init__(self, context):
        super(CdtGui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CdtGui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('cdt_rqt'), 'resource', 'CdtGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CdtGuiUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connects signals and slots
        self._widget.buttonState.clicked[bool].connect(self._slot_handle_button_state_clicked)

        # Initialize variables
        self.str_state = 'Start'
        self.state = 'idle'
        self.manual_goals_set = 0
        self.world_explored_percentage = 0
        self.objects = None

        # Initialize labels
        self._widget.buttonState.setText(self.str_state)
        self._widget.labelManualGoals.setText(str(self.manual_goals_set))
        self._widget.labelWorldExploredPercentage.setText(str(int(self.world_explored_percentage)) + '%')
        self._widget.labelDog.setEnabled(False)
        self._widget.labelComputer.setEnabled(False)
        self._widget.labelBarrel.setEnabled(False)
        self._widget.labelBarrow.setEnabled(False)
        self._widget.labelDuck.setEnabled(False)

        self._widget.labelDogPosition.setText("Not found")
        self._widget.labelComputerPosition.setText("Not found")
        self._widget.labelBarrelPosition.setText("Not found")
        self._widget.labelBarrowPosition.setText("Not found")
        self._widget.labelDuckPosition.setText("Not found")

        # Setup subscriberstr_system_status
        self.sub_objects      = rospy.Subscriber("/detected_objects", ObjectList, self.callback_objects)
        self.sub_manual_goal  = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_manual_goal)
        self.sub_manual_goal2 = rospy.Subscriber("/goal", PoseStamped, self.callback_manual_goal)
        self.sub_exp_space    = rospy.Subscriber("/explored_space_percentage", Int64, self.callback_explored_space_percentage)

        self.mutex_objects = Lock()
        self.mutex_world = Lock()
        self.mutex_goals = Lock()

        # Use timer to both keep track of time and update the window
        self.time_step = 15 # milliseconds
        self.stopwatch_time = 0.0 # to keep track of the challenge trial
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_display)
        self._update_plot_timer.start(self.time_step)


    # This is the only function / thread that updates the GUI.
    # Everything else just saves variables for this function to use.
    def update_display(self):
      # Update button
      self.update_button()

      # # Update manual goals
      self.update_goals()
      
      # Check status
      if self.state == 'idle':
        pass
      
      elif self.state == 'running': # the system is running
        # Update time
        self.stopwatch_time+= (self.time_step / (1000.0)) # update in seconds
        # Update label
        stopwatch_str = '{0:02.0f}:{1:05.2f}'.format(*divmod(self.stopwatch_time, 60))
        # print(self.stopwatch_time, stopwatch_str)
        self._widget.labelTime.setText(stopwatch_str)

        # Update objects
        self.update_objects()

        # Update world coverage
        self.update_world_coverage()

      elif self.str_state == 'finished':
        pass

    # Unregister subscribers and join threads when shutting down
    def shutdown_plugin(self):
        #self.sub_system_status.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    # Callback methods
    def callback_objects(self, msg):
        self.mutex_objects.acquire()
        self.objects = msg.objects
        self.mutex_objects.release()
        # self.update_objects()

    def callback_manual_goal(self, msg):
        self.mutex_goals.acquire()
        self.manual_goals_set = self.manual_goals_set + 1
        self.mutex_goals.release()
        # self.update_goals()
    
    def callback_explored_space_percentage(self, msg):
        self.mutex_world.acquire()
        self.world_explored_percentage = msg.data
        self.mutex_world.release()
        # self.update_world_coverage()
        
 
    # Slots (callbacks when an event is triggered)
    def do_service_call(self, service_name):
      try:
        request = ToggleExplorationRequest()
        request.button_press = True

        service = rospy.ServiceProxy(service_name, ToggleExploration)
        reply = service(request)
        return True
      except rospy.ServiceException as e:
        return False

    def _slot_handle_button_state_clicked(self):
      # Service call
      success = self.do_service_call('/world_explorer/start_stop_world_explorer')

      # Change state
      if self.str_state == 'Start':
        if success:
          self.str_state = 'Pause'
          self.state = 'running'

      elif self.str_state == 'Pause':
        if success:
          self.str_state = 'Resume'
          self.state = 'idle'

      elif self.str_state == 'Resume':
        if success:
          self.str_state = 'Pause'
          self.state = 'running'

      #
      #return self.do_empty_service_call('/cdt/set_idle')
    
    def update_button(self):
      self._widget.buttonState.setText(self.str_state)

      if self.state == 'idle':
        self._widget.buttonState.setStyleSheet("background-color: rgb(3, 157, 35); color: white;")

      elif self.state == 'running':
        self._widget.buttonState.setStyleSheet("background-color : rgb(255, 0, 0); color : white;")

      elif self.state == 'finished':
        self._widget.buttonState.setStyleSheet("background-color : rgb(0, 30, 255); color : white;")

    def update_goals(self):
      self.mutex_goals.acquire()
      self._widget.labelManualGoals.setText(str(self.manual_goals_set))
      self._widget.labelManualGoals.setStyleSheet("color: red;")
      self.mutex_goals.release()
    
    def update_world_coverage(self):
      self.mutex_world.acquire()
      self._widget.labelWorldExploredPercentage.setText(str(int(self.world_explored_percentage)) + '%')
      self._widget.labelWorldExploredPercentage.setStyleSheet("color: green;")
      self.mutex_world.release()
    
    def update_objects(self):
      self.mutex_objects.acquire()
      for obj in self.objects:
        if obj.id == 'dog':
          self._widget.labelDog.setEnabled(True)
          label = 'x: %.1f, y: %.1f' %(obj.position.x, obj.position.y)
          self._widget.labelDogPosition.setText(label)

        if obj.id == 'computer':
          self._widget.labelComputer.setEnabled(True)
          label = 'x: %.1f, y: %.1f' %(obj.position.x, obj.position.y)
          self._widget.labelComputerPosition.setText(label)

        if obj.id == 'barrel':
          self._widget.labelBarrel.setEnabled(True)
          label = 'x: %.1f, y: %.1f' %(obj.position.x, obj.position.y)
          self._widget.labelBarrelPosition.setText(label)

        if obj.id == 'barrow':
          self._widget.labelBarrow.setEnabled(True)
          label = 'x: %.1f, y: %.1f' %(obj.position.x, obj.position.y)
          self._widget.labelBarrowPosition.setText(label)

        if obj.id == 'duck':
          self._widget.labelDuck.setEnabled(True)
          label = 'x: %.1f, y: %.1f' %(obj.position.x, obj.position.y)
          self._widget.labelDuckPosition.setText(label)
        
      self.mutex_objects.release()
