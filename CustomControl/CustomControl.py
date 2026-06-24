import logging
import math
from typing import Optional

import qt
import ctk
import slicer
import vtk
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import parameterNodeWrapper


#
# Custom Control
#


class CustomControl(ScriptedLoadableModule):
    """Simple custom control module for SlicerROS2 workflows."""

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("Custom Control")
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "ROS 2")]
        self.parent.dependencies = ["ROS2"]
        self.parent.contributors = ["Mariana Bernardes (BWH)"]
        self.parent.helpText = _("""
This module provides a simple custom control interface for SlicerROS2 workflows.
It allows the user to select a loaded ROS2 robot, select or create a desired-pose
linear transform node, enter a ROS2 command topic, and send a pose command.
""")
        self.parent.acknowledgementText = _("""
Developed for custom robot integration workflows using 3D Slicer and SlicerROS2.
""")


#
# CustomControlParameterNode
#




class RefreshingTopicComboBox(qt.QComboBox):
    """Editable combo box that refreshes its item list when the popup opens.

    The editable text is preserved by the refresh callback, so a manually typed
    topic is not lost when the user opens the dropdown.
    """

    def __init__(self, parent=None):
        qt.QComboBox.__init__(self, parent)
        self.refreshCallback = None

    def showPopup(self):
        if callable(self.refreshCallback):
            self.refreshCallback()
        qt.QComboBox.showPopup(self)


@parameterNodeWrapper
class CustomControlParameterNode:
    """Persistent parameters for the Custom Control module.

    These values are saved with the MRML scene and restored on reload.
    The defaults are initialized for a Stewart-platform style workflow but remain
    editable in the GUI.
    """

    robotNodeID: str = ""
    desiredTransformNodeID: str = ""
    commandTopic: str = ""

    # Optional frame id for the PoseStamped header. If empty, the outgoing
    # PoseStamped header.frame_id is left empty and the robot controller is
    # expected to assume the command frame.
    frameId: str = ""

    # Generic trigger/bridge-enable control. Defaults are editable in the GUI.
    # Supported modes: "None", "Bool topic", "SetBoolString service".
    triggerMode: str = "SetBoolString service"
    triggerTopic: str = "airs/bridge_enable"
    triggerStatusTopic: str = "/airs/bridge_status"
    triggerEnabled: bool = False

    # JointState source used to populate the Joint Control tab.
    # AIRS actuator-state convention: /airs/state/joints.
    jointStateTopic: str = "/airs/state/joints"

    # Display conversion used in the Joint Control tab. The stored JointState
    # values remain unchanged; this only affects GUI display/edit fields.
    jointControlDisplayMode: str = "Length joints: m -> mm"


#
# CustomControlWidget
#


class CustomControlWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """User interface for the Custom Control module."""

    def __init__(self, parent=None) -> None:
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)
        self.logic: Optional[CustomControlLogic] = None
        self._parameterNode: Optional[CustomControlParameterNode] = None
        self.ros2Widget = None
        self._knownTransformNodeIDs = set()
        self._triggerStatusSubscriberNode = None
        self._triggerStatusObserverTag = None
        self._triggerStatusTopic = ""
        self._jointStateSubscriberNode = None
        self._jointStateObserverTag = None
        self._jointStateTopic = ""
        self._latestJointState = {"names": [], "positions": []}
        self._jointControlPlannedValues = {}
        self._jointControlCurrentRawValues = {}

    def setup(self) -> None:
        ScriptedLoadableModuleWidget.setup(self)

        self.logic = CustomControlLogic()

        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        self._setupCustomControlWidget()
        self._setupSlicerROS2CollapsibleSection()

        self.layout.addStretch(1)
        self.initializeParameterNode()

    def _setupCustomControlWidget(self) -> None:
        """Create the main Custom Control collapsible section.

        The layout intentionally mirrors the high-level organization of the
        ROS2 Motion Control module: a collapsible section with internal tabs.
        Only the Robot and 3D Control tabs are populated for now; the other
        tabs are placeholders for future functionality.
        """
        self.customControlCollapsibleButton = ctk.ctkCollapsibleButton()
        self.customControlCollapsibleButton.text = _("Custom Control")
        self.customControlCollapsibleButton.collapsed = False
        self.layout.addWidget(self.customControlCollapsibleButton)

        customLayout = qt.QVBoxLayout(self.customControlCollapsibleButton)
        customLayout.setContentsMargins(6, 6, 6, 6)
        customLayout.setSpacing(8)

        descriptionLabel = qt.QLabel(_(
            "Select a loaded robot, define a desired-pose transform, and publish "
            "that transform as a ROS2 PoseStamped command."
        ))
        descriptionLabel.wordWrap = True
        customLayout.addWidget(descriptionLabel)

        self.tabWidget = qt.QTabWidget()
        customLayout.addWidget(self.tabWidget)

        self.robotTab = qt.QWidget()
        self.jointControlTab = qt.QWidget()
        self.control3DTab = qt.QWidget()
        self.moveItTab = qt.QWidget()
        self.obstaclesTab = qt.QWidget()

        self.tabWidget.addTab(self.robotTab, _("Robot"))
        self.tabWidget.addTab(self.jointControlTab, _("Joint Control"))
        self.tabWidget.addTab(self.control3DTab, _("3D Control"))
        self.tabWidget.addTab(self.moveItTab, _("MoveIt"))
        self.tabWidget.addTab(self.obstaclesTab, _("Obstacles"))

        self._setupRobotTab()
        self._setupJointControlTab()
        self._setup3DControlTab()
        self._setupMoveItTab()
        self._setupObstaclesTab()

    def _setupRobotTab(self) -> None:
        robotLayout = qt.QVBoxLayout(self.robotTab)
        robotLayout.setContentsMargins(6, 6, 6, 6)
        robotLayout.setSpacing(8)

        robotDescriptionLabel = qt.QLabel(_(
            "Select one of the ROS2 robot nodes already loaded in the Slicer scene."
        ))
        robotDescriptionLabel.wordWrap = True
        robotLayout.addWidget(robotDescriptionLabel)

        formLayout = qt.QFormLayout()
        robotLayout.addLayout(formLayout)

        self.robotSelector = slicer.qMRMLNodeComboBox()
        self.robotSelector.nodeTypes = ["vtkMRMLROS2RobotNode"]
        self.robotSelector.selectNodeUponCreation = False
        self.robotSelector.addEnabled = False
        self.robotSelector.removeEnabled = False
        self.robotSelector.noneEnabled = True
        self.robotSelector.showHidden = False
        self.robotSelector.showChildNodeTypes = False
        self.robotSelector.setMRMLScene(slicer.mrmlScene)
        self.robotSelector.toolTip = _("Select one of the ROS2 robot nodes already loaded in the Slicer scene.")
        formLayout.addRow(_("Loaded robot:"), self.robotSelector)

        self.robotStatusLabel = qt.QLabel(_("Robot status: no robot selected."))
        self.robotStatusLabel.wordWrap = True
        robotLayout.addWidget(self.robotStatusLabel)

        self._setupTriggerControlSection(robotLayout)
        self._setupJointStateSection(robotLayout)
        robotLayout.addStretch(1)

        self.robotSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onRobotSelectionChanged)

    def _setupJointStateSection(self, parentLayout) -> None:
        """Create the JointState source section inside the Robot tab."""
        self.jointStateCollapsibleButton = ctk.ctkCollapsibleButton()
        self.jointStateCollapsibleButton.text = _("Joint State Source")
        self.jointStateCollapsibleButton.collapsed = False
        parentLayout.addWidget(self.jointStateCollapsibleButton)

        jointStateLayout = qt.QVBoxLayout(self.jointStateCollapsibleButton)
        jointStateLayout.setContentsMargins(6, 6, 6, 6)
        jointStateLayout.setSpacing(6)

        descriptionLabel = qt.QLabel(_(
            "Select the sensor_msgs/JointState topic that represents the robot-side joints. "
            "Detailed joint values are shown in the Joint Control tab. For AIRS actuator state, use /airs/state/joints."
        ))
        descriptionLabel.wordWrap = True
        jointStateLayout.addWidget(descriptionLabel)

        formLayout = qt.QFormLayout()
        jointStateLayout.addLayout(formLayout)

        self.jointStateTopicComboBox = RefreshingTopicComboBox()
        self.jointStateTopicComboBox.editable = True
        self.jointStateTopicComboBox.insertPolicy = qt.QComboBox.NoInsert
        self.jointStateTopicComboBox.refreshCallback = lambda: self.refreshJointStateTopicComboBox(preserveCurrent=True)
        self.jointStateTopicComboBox.lineEdit().placeholderText = "/robot/state/joints"
        self.jointStateTopicComboBox.toolTip = _(
            "Select an existing SlicerROS2 JointState subscriber topic, or type a topic to create "
            "a new JointState subscriber. The list refreshes when the dropdown opens."
        )
        formLayout.addRow(_("JointState topic:"), self.jointStateTopicComboBox)

        self.connectJointStateButton = qt.QPushButton(_("Subscribe to JointState"))
        self.connectJointStateButton.toolTip = _(
            "Create or reuse a SlicerROS2 JointState subscriber for the selected topic."
        )
        jointStateLayout.addWidget(self.connectJointStateButton)

        self.jointStateStatusLabel = qt.QLabel(_("Joint state source status: not subscribed."))
        self.jointStateStatusLabel.wordWrap = True
        jointStateLayout.addWidget(self.jointStateStatusLabel)

        self.jointStateSummaryLabel = qt.QLabel(_("Joints found: 0. Last message: none."))
        self.jointStateSummaryLabel.wordWrap = True
        jointStateLayout.addWidget(self.jointStateSummaryLabel)


        self.jointStateTopicComboBox.connect("currentTextChanged(QString)", self.onJointStateTopicChanged)
        self.jointStateTopicComboBox.lineEdit().connect("textChanged(QString)", self.onJointStateTopicChanged)
        self.connectJointStateButton.connect("clicked(bool)", self.onConnectJointStateButton)

    def _setupTriggerControlSection(self, parentLayout) -> None:
        """Create a generic trigger/bridge-enable control inside the Robot tab."""
        self.triggerCollapsibleButton = ctk.ctkCollapsibleButton()
        self.triggerCollapsibleButton.text = _("Command Enable")
        self.triggerCollapsibleButton.collapsed = False
        parentLayout.addWidget(self.triggerCollapsibleButton)

        triggerLayout = qt.QVBoxLayout(self.triggerCollapsibleButton)
        triggerLayout.setContentsMargins(6, 6, 6, 6)
        triggerLayout.setSpacing(6)

        triggerDescriptionLabel = qt.QLabel(_(
            "Configure the optional command-enable mechanism that allows or blocks robot commands. "
            "AIRS uses a SetBoolString service for bridge enable/disable; other robots may use no "
            "command enable mechanism or a Bool topic."
        ))
        triggerDescriptionLabel.wordWrap = True
        triggerLayout.addWidget(triggerDescriptionLabel)

        formLayout = qt.QFormLayout()
        triggerLayout.addLayout(formLayout)

        self.triggerModeComboBox = qt.QComboBox()
        self.triggerModeComboBox.addItems(["None", "Bool topic", "SetBoolString service"])
        self.triggerModeComboBox.toolTip = _(
            "Select how this robot enables/disables command execution. "
            "AIRS uses a SetBoolString service client in SlicerROS2. Use None if publishing the pose is sufficient."
        )
        formLayout.addRow(_("Command enable mode:"), self.triggerModeComboBox)

        self.triggerTopicComboBox = RefreshingTopicComboBox()
        self.triggerTopicComboBox.editable = True
        self.triggerTopicComboBox.insertPolicy = qt.QComboBox.NoInsert
        self.triggerTopicComboBox.refreshCallback = lambda: self.refreshTriggerNameComboBox(preserveCurrent=True)
        self.triggerTopicComboBox.lineEdit().placeholderText = "robot/bridge_enable"
        self.triggerTopicComboBox.toolTip = _(
            "Select an existing compatible SlicerROS2 service/topic, or type a new name. "
            "For SetBool service mode, the list shows existing SetBool service clients. "
            "For Bool topic mode, the list shows existing Bool publishers. "
            "A new client/publisher is created only when Apply Desired State is pressed."
        )
        self.triggerNameLabel = qt.QLabel(_("Enable service/topic:"))
        formLayout.addRow(self.triggerNameLabel, self.triggerTopicComboBox)

        self.triggerStatusTopicComboBox = RefreshingTopicComboBox()
        self.triggerStatusTopicComboBox.editable = True
        self.triggerStatusTopicComboBox.insertPolicy = qt.QComboBox.NoInsert
        self.triggerStatusTopicComboBox.refreshCallback = lambda: self.refreshTriggerStatusTopicComboBox(preserveCurrent=True)
        self.triggerStatusTopicComboBox.lineEdit().placeholderText = "/robot/bridge_status (optional)"
        self.triggerStatusTopicComboBox.toolTip = _(
            "Select an existing Bool status subscriber topic, or type a new status topic. "
            "A new Bool subscriber is created only when Subscribe to Status is pressed."
        )
        formLayout.addRow(_("Status topic (optional):"), self.triggerStatusTopicComboBox)

        self.subscribeTriggerStatusButton = qt.QPushButton(_("Subscribe to Status"))
        self.subscribeTriggerStatusButton.toolTip = _(
            "Create or reuse the Bool subscriber for the configured status topic. "
            "The module will not subscribe while you are typing the topic name."
        )
        triggerLayout.addWidget(self.subscribeTriggerStatusButton)

        self.triggerCurrentStatusLabel = qt.QLabel(_("Unknown"))
        self.triggerCurrentStatusLabel.wordWrap = True
        self.triggerCurrentStatusLabel.toolTip = _(
            "Current/reported command-enable state from the status topic. "
            "For AIRS this comes from /airs/bridge_status."
        )
        formLayout.addRow(_("Current status:"), self.triggerCurrentStatusLabel)

        self.triggerEnableCheckBox = qt.QCheckBox(_("Enable commands"))
        self.triggerEnableCheckBox.checked = False
        self.triggerEnableCheckBox.toolTip = _(
            "Desired command-enable value to send when Apply Desired State is pressed. "
            "Incoming status messages update only the Current status label and do not overwrite this checkbox."
        )
        formLayout.addRow(_("Desired state:"), self.triggerEnableCheckBox)

        self.publishTriggerButton = qt.QPushButton(_("Apply Desired State"))
        self.publishTriggerButton.toolTip = _(
            "Apply the requested command-enable state using the selected mode. For AIRS, this calls the SetBoolString service client."
        )
        triggerLayout.addWidget(self.publishTriggerButton)

        self.triggerStatusLabel = qt.QLabel(_("Command enable status: enter a status topic and click Subscribe to Status when ready."))
        self.triggerStatusLabel.wordWrap = True
        triggerLayout.addWidget(self.triggerStatusLabel)

        self.triggerModeComboBox.connect("currentTextChanged(QString)", self.onTriggerModeChanged)
        self.triggerModeComboBox.connect("currentIndexChanged(int)", self.onTriggerModeChanged)
        self.triggerTopicComboBox.connect("currentTextChanged(QString)", self.onTriggerTopicChanged)
        self.triggerTopicComboBox.lineEdit().connect("textChanged(QString)", self.onTriggerTopicChanged)
        self.triggerStatusTopicComboBox.connect("currentTextChanged(QString)", self.onTriggerStatusTopicChanged)
        self.triggerStatusTopicComboBox.lineEdit().connect("textChanged(QString)", self.onTriggerStatusTopicChanged)
        self.subscribeTriggerStatusButton.connect("clicked(bool)", self.onSubscribeTriggerStatusButton)
        self.triggerEnableCheckBox.connect("toggled(bool)", self.onTriggerEnabledChanged)
        self.publishTriggerButton.connect("clicked(bool)", self.onPublishTriggerButton)
        self._updateTriggerModeUi()

    def _setupJointControlTab(self) -> None:
        layout = qt.QVBoxLayout(self.jointControlTab)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)

        descriptionLabel = qt.QLabel(_(
            "Joint Control uses the JointState source selected in the Robot tab. "
            "Current joint values are always shown as raw JointState.position values. "
            "Planned joint values and FK preview are optional and will only be enabled for robots with compatible FK support."
        ))
        descriptionLabel.wordWrap = True
        layout.addWidget(descriptionLabel)

        # Stage 1: current joint state, available for any robot that publishes JointState.
        self.currentJointStateCollapsibleButton = ctk.ctkCollapsibleButton()
        self.currentJointStateCollapsibleButton.text = _("Current Joint State")
        self.currentJointStateCollapsibleButton.collapsed = False
        layout.addWidget(self.currentJointStateCollapsibleButton)

        currentLayout = qt.QVBoxLayout(self.currentJointStateCollapsibleButton)
        currentLayout.setContentsMargins(6, 6, 6, 6)
        currentLayout.setSpacing(6)

        currentFormLayout = qt.QFormLayout()
        currentLayout.addLayout(currentFormLayout)

        self.jointControlSourceTopicLabel = qt.QLabel(_("Not subscribed"))
        self.jointControlSourceTopicLabel.wordWrap = True
        currentFormLayout.addRow(_("JointState source:"), self.jointControlSourceTopicLabel)

        self.jointControlStatusLabel = qt.QLabel(_(
            "Current joint state: subscribe to a JointState topic in the Robot tab."
        ))
        self.jointControlStatusLabel.wordWrap = True
        currentFormLayout.addRow(_("Status:"), self.jointControlStatusLabel)

        self.currentJointStateTableWidget = qt.QTableWidget()
        self.currentJointStateTableWidget.setColumnCount(2)
        self.currentJointStateTableWidget.setHorizontalHeaderLabels([
            _("Joint name"),
            _("Current raw value"),
        ])
        self.currentJointStateTableWidget.horizontalHeader().setStretchLastSection(False)
        self.currentJointStateTableWidget.horizontalHeader().setSectionResizeMode(0, qt.QHeaderView.Stretch)
        self.currentJointStateTableWidget.horizontalHeader().setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        self.currentJointStateTableWidget.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        self.currentJointStateTableWidget.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
        self.currentJointStateTableWidget.minimumHeight = 180
        currentLayout.addWidget(self.currentJointStateTableWidget)

        # Stage 2: planned joint state / FK preview. Kept separate and disabled until
        # a compatible robot-specific FK path is implemented or configured.
        self.plannedJointStateCollapsibleButton = ctk.ctkCollapsibleButton()
        self.plannedJointStateCollapsibleButton.text = _("Planned Joint State / FK Preview")
        self.plannedJointStateCollapsibleButton.collapsed = True
        layout.addWidget(self.plannedJointStateCollapsibleButton)

        plannedLayout = qt.QVBoxLayout(self.plannedJointStateCollapsibleButton)
        plannedLayout.setContentsMargins(6, 6, 6, 6)
        plannedLayout.setSpacing(6)

        plannedDescriptionLabel = qt.QLabel(_(
            "Optional stage for robots that can compute forward kinematics from planned joint/actuator values. "
            "Disabled until an FK service or robot-specific FK adapter is configured."
        ))
        plannedDescriptionLabel.wordWrap = True
        plannedLayout.addWidget(plannedDescriptionLabel)

        plannedFormLayout = qt.QFormLayout()
        plannedLayout.addLayout(plannedFormLayout)

        self.fkServiceLineEdit = qt.QLineEdit()
        self.fkServiceLineEdit.text = "airs/fk"
        self.fkServiceLineEdit.placeholderText = "robot/fk"
        self.fkServiceLineEdit.enabled = False
        plannedFormLayout.addRow(_("FK service:"), self.fkServiceLineEdit)

        self.plannedJointStateStatusLabel = qt.QLabel(_("Planned/FK stage: unavailable for the selected robot."))
        self.plannedJointStateStatusLabel.wordWrap = True
        plannedFormLayout.addRow(_("Status:"), self.plannedJointStateStatusLabel)

        self.plannedJointStateTableWidget = qt.QTableWidget()
        self.plannedJointStateTableWidget.setColumnCount(3)
        self.plannedJointStateTableWidget.setHorizontalHeaderLabels([
            _("Joint name"),
            _("Current raw value"),
            _("Planned raw value"),
        ])
        self.plannedJointStateTableWidget.horizontalHeader().setSectionResizeMode(0, qt.QHeaderView.Stretch)
        self.plannedJointStateTableWidget.horizontalHeader().setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        self.plannedJointStateTableWidget.horizontalHeader().setSectionResizeMode(2, qt.QHeaderView.ResizeToContents)
        self.plannedJointStateTableWidget.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        self.plannedJointStateTableWidget.minimumHeight = 160
        self.plannedJointStateTableWidget.enabled = False
        plannedLayout.addWidget(self.plannedJointStateTableWidget)

        buttonLayout = qt.QHBoxLayout()
        plannedLayout.addLayout(buttonLayout)

        self.setPlannedFromCurrentButton = qt.QPushButton(_("Set planned from current"))
        self.setPlannedFromCurrentButton.enabled = False
        self.setPlannedFromCurrentButton.toolTip = _(
            "Copy the latest current raw JointState values into the planned column. "
            "This becomes available when FK preview is enabled."
        )
        buttonLayout.addWidget(self.setPlannedFromCurrentButton)

        self.clearPlannedJointValuesButton = qt.QPushButton(_("Clear planned"))
        self.clearPlannedJointValuesButton.enabled = False
        self.clearPlannedJointValuesButton.toolTip = _(
            "Clear the planned values without changing the current state subscription."
        )
        buttonLayout.addWidget(self.clearPlannedJointValuesButton)
        buttonLayout.addStretch(1)

        self.plannedJointStateTableWidget.connect(
            "cellChanged(int,int)", self.onJointControlTableCellChanged
        )
        self.setPlannedFromCurrentButton.connect("clicked(bool)", self.onSetPlannedFromCurrentButton)
        self.clearPlannedJointValuesButton.connect("clicked(bool)", self.onClearPlannedJointValuesButton)

        layout.addStretch(1)

    def _setup3DControlTab(self) -> None:
        controlLayout = qt.QVBoxLayout(self.control3DTab)
        controlLayout.setContentsMargins(6, 6, 6, 6)
        controlLayout.setSpacing(8)

        controlDescriptionLabel = qt.QLabel(_(
            "Select or create an interactive desired-pose transform, choose an existing "
            "SlicerROS2 PoseStamped publisher topic, or type a new topic name to create "
            "a new PoseStamped publisher when sending the command."
        ))
        controlDescriptionLabel.wordWrap = True
        controlLayout.addWidget(controlDescriptionLabel)

        formLayout = qt.QFormLayout()
        controlLayout.addLayout(formLayout)

        self.desiredTransformSelector = slicer.qMRMLNodeComboBox()
        self.desiredTransformSelector.nodeTypes = ["vtkMRMLLinearTransformNode"]
        self.desiredTransformSelector.selectNodeUponCreation = True
        self.desiredTransformSelector.addEnabled = True
        self.desiredTransformSelector.removeEnabled = False
        self.desiredTransformSelector.noneEnabled = True
        self.desiredTransformSelector.renameEnabled = True
        self.desiredTransformSelector.showHidden = False
        self.desiredTransformSelector.showChildNodeTypes = False
        self.desiredTransformSelector.baseName = "DesiredRobotPose"
        self.desiredTransformSelector.setMRMLScene(slicer.mrmlScene)
        self.desiredTransformSelector.toolTip = _(
            "Select or create the linear transform node representing the desired robot pose. "
            "When selected, it is made visible and interactive in the viewers."
        )
        formLayout.addRow(_("Desired pose transform:"), self.desiredTransformSelector)
        self._initializeKnownTransformNodeIDs()

        self.commandTopicComboBox = RefreshingTopicComboBox()
        self.commandTopicComboBox.editable = True
        self.commandTopicComboBox.insertPolicy = qt.QComboBox.NoInsert
        self.commandTopicComboBox.refreshCallback = lambda: self.refreshCommandTopicComboBox(preserveCurrent=True)
        self.commandTopicComboBox.lineEdit().placeholderText = "/robot/command/pose"
        self.commandTopicComboBox.toolTip = _(
            "Select an existing PoseStamped publisher from the SlicerROS2 Topics tab, "
            "or type a new topic name to create a new PoseStamped publisher from Slicer. "
            "The list refreshes automatically when you open the dropdown."
        )
        formLayout.addRow(_("Command pose topic:"), self.commandTopicComboBox)

        self.frameIdLineEdit = qt.QLineEdit()
        self.frameIdLineEdit.text = ""
        self.frameIdLineEdit.placeholderText = "optional, e.g., base_link"
        self.frameIdLineEdit.toolTip = _(
            "Optional. ROS frame in which the desired pose is expressed. "
            "If empty, PoseStamped.header.frame_id is left empty and the robot controller "
            "must assume the command frame."
        )
        formLayout.addRow(_("Command frame ID (optional):"), self.frameIdLineEdit)

        self.sendCommandButton = qt.QPushButton(_("Send Command"))
        self.sendCommandButton.toolTip = _(
            "Create or reuse a PoseStamped publisher for the selected topic, then publish the selected transform."
        )
        controlLayout.addWidget(self.sendCommandButton)

        self.statusLabel = qt.QLabel(_("Status: select a robot and a desired-pose transform."))
        self.statusLabel.wordWrap = True
        controlLayout.addWidget(self.statusLabel)
        controlLayout.addStretch(1)

        self.desiredTransformSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onDesiredTransformSelectionChanged)
        self.commandTopicComboBox.connect("currentTextChanged(QString)", self.onCommandTopicChanged)
        self.commandTopicComboBox.lineEdit().connect("textChanged(QString)", self.onCommandTopicChanged)
        self.frameIdLineEdit.connect("textChanged(QString)", self.onFrameIdChanged)
        self.sendCommandButton.connect("clicked(bool)", self.onSendCommandButton)

    def _setupMoveItTab(self) -> None:
        layout = qt.QVBoxLayout(self.moveItTab)
        layout.setContentsMargins(6, 6, 6, 6)
        placeholder = qt.QLabel(_("MoveIt options will be added here."))
        placeholder.wordWrap = True
        layout.addWidget(placeholder)
        layout.addStretch(1)

    def _setupObstaclesTab(self) -> None:
        layout = qt.QVBoxLayout(self.obstaclesTab)
        layout.setContentsMargins(6, 6, 6, 6)
        placeholder = qt.QLabel(_("Obstacle/planning-scene options will be added here."))
        placeholder.wordWrap = True
        layout.addWidget(placeholder)
        layout.addStretch(1)

    def _setupSlicerROS2CollapsibleSection(self) -> None:
        self.ros2CollapsibleButton = ctk.ctkCollapsibleButton()
        self.ros2CollapsibleButton.text = _("Slicer ROS2")
        self.ros2CollapsibleButton.collapsed = True
        self.layout.addWidget(self.ros2CollapsibleButton)

        rosLayout = qt.QVBoxLayout(self.ros2CollapsibleButton)
        rosLayout.setContentsMargins(6, 6, 6, 6)

        try:
            self.ros2Widget = slicer.modules.ros2.createNewWidgetRepresentation()
            self.ros2Widget.setMRMLScene(slicer.mrmlScene)
            rosLayout.addWidget(self.ros2Widget)
        except Exception as exc:
            warningLabel = qt.QLabel(_(f"Could not embed the Slicer ROS2 module: {exc}"))
            warningLabel.wordWrap = True
            rosLayout.addWidget(warningLabel)
            logging.warning(f"Could not embed the Slicer ROS2 module: {exc}")

    def cleanup(self) -> None:
        self._disconnectTriggerStatusSubscriber()
        self._disconnectJointStateSubscriber()
        self.removeObservers()

    def enter(self) -> None:
        self.initializeParameterNode()

    def exit(self) -> None:
        pass

    def onSceneStartClose(self, caller, event) -> None:
        self._disconnectTriggerStatusSubscriber()
        self._disconnectJointStateSubscriber()
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        if self.parent.isEntered:
            self._initializeKnownTransformNodeIDs()
            self.initializeParameterNode()

    def initializeParameterNode(self) -> None:
        self.setParameterNode(self.logic.getParameterNode())

    def setParameterNode(self, inputParameterNode: Optional[CustomControlParameterNode]) -> None:
        self._parameterNode = inputParameterNode
        self._syncGuiFromParameterNode()
        self._updateSendCommandButtonState()

    def _syncGuiFromParameterNode(self) -> None:
        if not self._parameterNode:
            return

        if self._parameterNode.robotNodeID:
            robotNode = slicer.mrmlScene.GetNodeByID(self._parameterNode.robotNodeID)
            if robotNode:
                self.robotSelector.setCurrentNode(robotNode)

        if self._parameterNode.desiredTransformNodeID:
            transformNode = slicer.mrmlScene.GetNodeByID(self._parameterNode.desiredTransformNodeID)
            if transformNode:
                self.desiredTransformSelector.setCurrentNode(transformNode)

        self.refreshCommandTopicComboBox(preserveCurrent=True)
        self._setCommandTopicComboText(self._parameterNode.commandTopic or "")
        self.frameIdLineEdit.text = self._parameterNode.frameId or ""
        triggerMode = self._parameterNode.triggerMode or "SetBoolString service"
        index = self.triggerModeComboBox.findText(triggerMode)
        self.triggerModeComboBox.blockSignals(True)
        self.triggerModeComboBox.setCurrentIndex(index if index >= 0 else self.triggerModeComboBox.findText("SetBoolString service"))
        self.triggerModeComboBox.blockSignals(False)
        self.refreshTriggerNameComboBox(preserveCurrent=True)
        self._setTriggerNameComboText(self._parameterNode.triggerTopic or "airs/bridge_enable")
        self.refreshTriggerStatusTopicComboBox(preserveCurrent=True)
        self._setTriggerStatusTopicComboText(self._parameterNode.triggerStatusTopic or "/airs/bridge_status")
        self.triggerEnableCheckBox.checked = bool(self._parameterNode.triggerEnabled)
        self.refreshJointStateTopicComboBox(preserveCurrent=True)
        self._setJointStateTopicComboText(self._parameterNode.jointStateTopic or "/airs/state/joints")
        self._updateTriggerModeUi()

    def _updateSendCommandButtonState(self) -> None:
        robotNode = self.robotSelector.currentNode()
        desiredTransformNode = self.desiredTransformSelector.currentNode()
        commandTopic = self._currentCommandTopicText()
        canSend = bool(robotNode and desiredTransformNode and commandTopic)
        self.sendCommandButton.enabled = canSend
        if hasattr(self, "publishTriggerButton"):
            triggerMode = self.triggerModeComboBox.currentText if hasattr(self, "triggerModeComboBox") else "None"
            triggerName = self._currentTriggerNameText() if hasattr(self, "triggerTopicComboBox") else ""
            self.publishTriggerButton.enabled = bool(robotNode and (triggerMode == "None" or triggerName))
        if canSend:
            self.sendCommandButton.toolTip = _(
                "Publish the selected desired-pose transform as geometry_msgs/msg/PoseStamped. "
                "If a publisher already exists for this topic, it will be reused. "
                "The frame ID is optional."
            )
        else:
            self.sendCommandButton.toolTip = _(
                "Select a robot, select or create a desired-pose transform, and enter a command topic. "
                "The frame ID is optional."
            )

    def _initializeKnownTransformNodeIDs(self) -> None:
        """Remember transform nodes that already exist, so only newly created pose nodes are auto-initialized."""
        self._knownTransformNodeIDs = set()
        self._triggerStatusSubscriberNode = None
        self._triggerStatusObserverTag = None
        self._triggerStatusTopic = ""
        scene = slicer.mrmlScene
        if scene is None:
            return
        for index in range(scene.GetNumberOfNodesByClass("vtkMRMLLinearTransformNode")):
            node = scene.GetNthNodeByClass(index, "vtkMRMLLinearTransformNode")
            if node is not None:
                self._knownTransformNodeIDs.add(node.GetID())

    def _isNewlyCreatedDesiredTransform(self, transformNode) -> bool:
        """Return True only the first time a transform node appears after module setup."""
        if transformNode is None:
            return False
        nodeID = transformNode.GetID()
        if not nodeID:
            return False
        if nodeID in self._knownTransformNodeIDs:
            return False
        self._knownTransformNodeIDs.add(nodeID)
        return True

    def _initializeDesiredTransformFromCurrentRobotPose(self, transformNode) -> bool:
        """Initialize a newly created desired-pose transform from the selected robot's current tip pose."""
        if transformNode is None or self.logic is None:
            return False
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        if robotNode is None:
            return False
        matrix = self.logic.getCurrentRobotTipWorldMatrix(robotNode)
        if matrix is None:
            return False
        transformNode.SetMatrixTransformToParent(matrix)
        transformNode.Modified()
        return True

    def _makeTransformInteractiveInViews(self, transformNode) -> None:
        """Show the selected transform and enable interactive transform handles in the viewers."""
        if transformNode is None:
            return

        displayNode = transformNode.GetDisplayNode()
        if displayNode is None:
            displayNode = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLTransformDisplayNode",
                f"{transformNode.GetName()}_Display"
            )
            transformNode.SetAndObserveDisplayNodeID(displayNode.GetID())

        # Make the transform visible in the scene and enable the interactive
        # transform editor handles in 3D views. These methods are present in
        # current Slicer transform display nodes; hasattr keeps this safe across
        # Slicer versions.
        displayNode.SetVisibility(True)
        if hasattr(displayNode, "SetVisibility2D"):
            displayNode.SetVisibility2D(True)
        if hasattr(displayNode, "SetVisibility3D"):
            displayNode.SetVisibility3D(True)
        if hasattr(displayNode, "SetEditorVisibility"):
            displayNode.SetEditorVisibility(True)

        transformNode.Modified()
        displayNode.Modified()

    def onRobotSelectionChanged(self, node) -> None:
        if self._parameterNode:
            self._parameterNode.robotNodeID = node.GetID() if node else ""
        if hasattr(self, "robotStatusLabel"):
            if node:
                self.robotStatusLabel.text = _(f"Robot status: selected '{node.GetName()}'.")
            else:
                self.robotStatusLabel.text = _("Robot status: no robot selected.")
        self.refreshCommandTopicComboBox(preserveCurrent=True)
        self.refreshTriggerNameComboBox(preserveCurrent=True)
        self.refreshTriggerStatusTopicComboBox(preserveCurrent=True)
        self.refreshJointStateTopicComboBox(preserveCurrent=True)
        self._updateSendCommandButtonState()

    def onDesiredTransformSelectionChanged(self, node) -> None:
        if self._parameterNode:
            self._parameterNode.desiredTransformNodeID = node.GetID() if node else ""
        if node:
            initializedFromRobot = False
            if self._isNewlyCreatedDesiredTransform(node):
                initializedFromRobot = self._initializeDesiredTransformFromCurrentRobotPose(node)

            self._makeTransformInteractiveInViews(node)
            if initializedFromRobot:
                self.statusLabel.text = _(
                    f"Status: new transform '{node.GetName()}' initialized from the current robot pose and made interactive."
                )
            else:
                self.statusLabel.text = _(
                    f"Status: transform '{node.GetName()}' selected and made interactive in the viewers."
                )
        self._updateSendCommandButtonState()

    def _currentCommandTopicText(self) -> str:
        if not hasattr(self, "commandTopicComboBox"):
            return ""
        return str(self.commandTopicComboBox.currentText).strip()

    def _setCommandTopicComboText(self, topic: str) -> None:
        """Set the editable text without adding a fake dropdown item.

        The dropdown items should represent only existing SlicerROS2 publisher
        nodes. A manually typed topic is kept in the editor text and will create
        a new PoseStamped publisher when Send Command is pressed.
        """
        if not hasattr(self, "commandTopicComboBox"):
            return
        topic = str(topic or "").strip()
        index = self.commandTopicComboBox.findText(topic) if topic else -1
        self.commandTopicComboBox.blockSignals(True)
        if index >= 0:
            self.commandTopicComboBox.setCurrentIndex(index)
        elif self.commandTopicComboBox.isEditable() and self.commandTopicComboBox.lineEdit() is not None:
            self.commandTopicComboBox.setCurrentIndex(-1)
            self.commandTopicComboBox.lineEdit().setText(topic)
        self.commandTopicComboBox.blockSignals(False)

    def refreshCommandTopicComboBox(self, preserveCurrent: bool = True) -> None:
        if not hasattr(self, "commandTopicComboBox") or self.logic is None:
            return
        currentText = self._currentCommandTopicText() if preserveCurrent else ""
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        topics = self.logic.listPoseStampedPublisherTopics(robotNode)

        # The dropdown list should contain only existing SlicerROS2 publisher
        # nodes. Do not add defaults or the user's typed text as list entries.
        publisherTopics = []
        for topic in topics:
            topic = str(topic or "").strip()
            if topic and topic not in publisherTopics:
                publisherTopics.append(topic)

        self.commandTopicComboBox.blockSignals(True)
        self.commandTopicComboBox.clear()
        for topic in publisherTopics:
            self.commandTopicComboBox.addItem(topic)

        if currentText:
            index = self.commandTopicComboBox.findText(currentText)
            if index >= 0:
                self.commandTopicComboBox.setCurrentIndex(index)
            elif self.commandTopicComboBox.isEditable() and self.commandTopicComboBox.lineEdit() is not None:
                self.commandTopicComboBox.setCurrentIndex(-1)
                self.commandTopicComboBox.lineEdit().setText(currentText)
        elif publisherTopics:
            self.commandTopicComboBox.setCurrentIndex(0)
        elif self.commandTopicComboBox.isEditable() and self.commandTopicComboBox.lineEdit() is not None:
            self.commandTopicComboBox.setCurrentIndex(-1)
            self.commandTopicComboBox.lineEdit().clear()

        self.commandTopicComboBox.blockSignals(False)

    def onRefreshCommandTopicsButton(self, checked=False) -> None:
        # Kept as a small utility method for scripted use. The GUI no longer
        # exposes a Refresh button; the topic combo refreshes on popup open.
        self.refreshCommandTopicComboBox(preserveCurrent=True)
        self._updateSendCommandButtonState()

    def onCommandTopicChanged(self, text) -> None:
        if self._parameterNode:
            self._parameterNode.commandTopic = str(text).strip()
        self._updateSendCommandButtonState()

    def onFrameIdChanged(self, text) -> None:
        if self._parameterNode:
            self._parameterNode.frameId = str(text).strip()
        self._updateSendCommandButtonState()

    def _currentJointStateTopicText(self) -> str:
        if not hasattr(self, "jointStateTopicComboBox"):
            return ""
        return str(self.jointStateTopicComboBox.currentText).strip()

    def _setJointStateTopicComboText(self, topic: str) -> None:
        if not hasattr(self, "jointStateTopicComboBox"):
            return
        topic = str(topic or "").strip()
        index = self.jointStateTopicComboBox.findText(topic) if topic else -1
        self.jointStateTopicComboBox.blockSignals(True)
        if index >= 0:
            self.jointStateTopicComboBox.setCurrentIndex(index)
        elif self.jointStateTopicComboBox.isEditable() and self.jointStateTopicComboBox.lineEdit() is not None:
            self.jointStateTopicComboBox.setCurrentIndex(-1)
            self.jointStateTopicComboBox.lineEdit().setText(topic)
        self.jointStateTopicComboBox.blockSignals(False)

    def refreshJointStateTopicComboBox(self, preserveCurrent: bool = True) -> None:
        if not hasattr(self, "jointStateTopicComboBox") or self.logic is None:
            return
        currentText = self._currentJointStateTopicText() if preserveCurrent else ""
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        topics = self.logic.listJointStateSubscriberTopics(robotNode)

        subscriberTopics = []
        for topic in topics:
            topic = str(topic or "").strip()
            if topic and topic not in subscriberTopics:
                subscriberTopics.append(topic)

        self.jointStateTopicComboBox.blockSignals(True)
        self.jointStateTopicComboBox.clear()
        for topic in subscriberTopics:
            self.jointStateTopicComboBox.addItem(topic)

        if currentText:
            index = self.jointStateTopicComboBox.findText(currentText)
            if index >= 0:
                self.jointStateTopicComboBox.setCurrentIndex(index)
            elif self.jointStateTopicComboBox.isEditable() and self.jointStateTopicComboBox.lineEdit() is not None:
                self.jointStateTopicComboBox.setCurrentIndex(-1)
                self.jointStateTopicComboBox.lineEdit().setText(currentText)
        elif subscriberTopics:
            self.jointStateTopicComboBox.setCurrentIndex(0)
        elif self.jointStateTopicComboBox.isEditable() and self.jointStateTopicComboBox.lineEdit() is not None:
            self.jointStateTopicComboBox.setCurrentIndex(-1)
            self.jointStateTopicComboBox.lineEdit().clear()
        self.jointStateTopicComboBox.blockSignals(False)

    def onJointStateTopicChanged(self, text) -> None:
        if self._parameterNode:
            self._parameterNode.jointStateTopic = str(text).strip()

    def _disconnectJointStateSubscriber(self) -> None:
        if self._jointStateSubscriberNode is not None and self._jointStateObserverTag is not None:
            try:
                self._jointStateSubscriberNode.RemoveObserver(self._jointStateObserverTag)
            except Exception:
                pass
        self._jointStateSubscriberNode = None
        self._jointStateObserverTag = None
        self._jointStateTopic = ""
        if hasattr(self, "jointControlSourceTopicLabel"):
            self.jointControlSourceTopicLabel.text = _("Not subscribed")
        if hasattr(self, "jointStateStatusLabel"):
            self.jointStateStatusLabel.text = _("Joint state source status: not subscribed.")
        if hasattr(self, "jointStateSummaryLabel"):
            self.jointStateSummaryLabel.text = _("Joints found: 0. Last message: none.")
        if hasattr(self, "currentJointStateTableWidget"):
            self.currentJointStateTableWidget.setRowCount(0)
        if hasattr(self, "plannedJointStateTableWidget"):
            self.plannedJointStateTableWidget.setRowCount(0)

    def onConnectJointStateButton(self, checked=False) -> None:
        self._connectJointStateSubscriber()

    def _connectJointStateSubscriber(self) -> None:
        if self.logic is None or not hasattr(self, "jointStateTopicComboBox"):
            return
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        topic = self._currentJointStateTopicText()

        if robotNode is None:
            self._disconnectJointStateSubscriber()
            if hasattr(self, "jointStateStatusLabel"):
                self.jointStateStatusLabel.text = _("Joint state source status: select a robot before subscribing.")
            return
        if not topic:
            self._disconnectJointStateSubscriber()
            if hasattr(self, "jointStateStatusLabel"):
                self.jointStateStatusLabel.text = _("Joint state source status: enter a JointState topic.")
            return

        normalizedTopic = self.logic._normalizeTopicName(topic)
        if (self._jointStateSubscriberNode is not None
                and self._jointStateTopic == normalizedTopic):
            self.jointStateStatusLabel.text = _(f"Joint state source status: already listening to {normalizedTopic}.")
            return

        self._disconnectJointStateSubscriber()
        try:
            subscriber, normalizedTopic, action = self.logic.getOrCreateJointStateSubscriber(robotNode, topic)
            self._jointStateSubscriberNode = subscriber
            self._jointStateTopic = normalizedTopic
            self._jointStateObserverTag = subscriber.AddObserver(vtk.vtkCommand.ModifiedEvent, self._onJointStateReceived)
            self._setJointStateTopicComboText(normalizedTopic)
            self.jointStateStatusLabel.text = _(f"Joint state source status: listening to {normalizedTopic} ({action} subscriber).")
            self._onJointStateReceived(subscriber, vtk.vtkCommand.ModifiedEvent)
        except Exception as exc:
            self._disconnectJointStateSubscriber()
            self.jointStateStatusLabel.text = _(f"Joint state source status: could not subscribe: {exc}")
            logging.exception("Failed to subscribe to JointState topic")

    def _onJointStateReceived(self, caller, event) -> None:
        try:
            message = caller.GetLastMessage()
            names, positions = self.logic.jointStateNamesAndPositions(message) if self.logic is not None else ([], [])
        except Exception:
            names, positions = [], []

        self._latestJointState = {"names": list(names), "positions": list(positions)}
        self._updateJointStateTable(names, positions)
        self._updateJointControlFromJointState(names, positions)

    def _updateJointStateTable(self, names, positions) -> None:
        """Update the compact Joint State Source feedback in the Robot tab.

        Detailed values are intentionally shown only in the Joint Control tab.
        """
        names = list(names or [])
        positions = list(positions or [])
        count = min(len(names), len(positions))

        if hasattr(self, "jointStateSummaryLabel"):
            if count:
                self.jointStateSummaryLabel.text = _(f"Joints found: {count}. Last message received from JointState topic.")
            else:
                self.jointStateSummaryLabel.text = _("Joints found: 0. Last message: none or invalid JointState message.")

        if hasattr(self, "jointStateStatusLabel") and count:
            topic = self._jointStateTopic or self._currentJointStateTopicText()
            self.jointStateStatusLabel.text = _(f"Joint state source status: receiving {count} joints from {topic}.")

    def _formatJointControlValue(self, rawValue) -> str:
        try:
            return f"{float(rawValue):.6g}"
        except Exception:
            return ""

    def _plannedRawValueForJoint(self, jointName: str, currentRawValue):
        if jointName in self._jointControlPlannedValues:
            return self._jointControlPlannedValues[jointName]
        return self._formatJointControlValue(currentRawValue)

    def _updateJointControlFromJointState(self, names, positions) -> None:
        names = list(names or [])
        positions = list(positions or [])
        count = min(len(names), len(positions))

        if hasattr(self, "jointControlSourceTopicLabel"):
            topic = self._jointStateTopic or self._currentJointStateTopicText()
            self.jointControlSourceTopicLabel.text = topic if topic else _("Not subscribed")

        if hasattr(self, "currentJointStateTableWidget"):
            self.currentJointStateTableWidget.blockSignals(True)
            self.currentJointStateTableWidget.setRowCount(count)
            for row in range(count):
                jointName = str(names[row])
                rawValue = float(positions[row])
                self._jointControlCurrentRawValues[jointName] = rawValue

                nameItem = qt.QTableWidgetItem(jointName)
                nameItem.setFlags(nameItem.flags() & ~qt.Qt.ItemIsEditable)

                currentItem = qt.QTableWidgetItem(self._formatJointControlValue(rawValue))
                currentItem.setFlags(currentItem.flags() & ~qt.Qt.ItemIsEditable)

                self.currentJointStateTableWidget.setItem(row, 0, nameItem)
                self.currentJointStateTableWidget.setItem(row, 1, currentItem)
            self.currentJointStateTableWidget.blockSignals(False)

        # Keep the optional planned/FK table synchronized with the same joints.
        # It remains disabled until FK support is explicitly enabled later.
        if hasattr(self, "plannedJointStateTableWidget"):
            self.plannedJointStateTableWidget.blockSignals(True)
            self.plannedJointStateTableWidget.setRowCount(count)
            for row in range(count):
                jointName = str(names[row])
                rawValue = float(positions[row])

                nameItem = qt.QTableWidgetItem(jointName)
                nameItem.setFlags(nameItem.flags() & ~qt.Qt.ItemIsEditable)

                currentItem = qt.QTableWidgetItem(self._formatJointControlValue(rawValue))
                currentItem.setFlags(currentItem.flags() & ~qt.Qt.ItemIsEditable)

                plannedItem = qt.QTableWidgetItem(self._plannedRawValueForJoint(jointName, rawValue))
                plannedItem.setToolTip(_("Editable planned raw value. This stage is disabled until FK preview is available."))

                self.plannedJointStateTableWidget.setItem(row, 0, nameItem)
                self.plannedJointStateTableWidget.setItem(row, 1, currentItem)
                self.plannedJointStateTableWidget.setItem(row, 2, plannedItem)
            self.plannedJointStateTableWidget.blockSignals(False)

        if hasattr(self, "jointControlStatusLabel"):
            if count:
                self.jointControlStatusLabel.text = _(
                    f"Current joint state: receiving {count} joints. Values are raw JointState.position values."
                )
            else:
                self.jointControlStatusLabel.text = _(
                    "Current joint state: no JointState values received yet."
                )

    def onJointControlDisplayModeChanged(self, text) -> None:
        # Kept for backward compatibility with older saved scenes.
        # Joint Control now displays raw JointState.position values only.
        pass

    def onJointControlTableCellChanged(self, row: int, column: int) -> None:
        if column != 2 or not hasattr(self, "plannedJointStateTableWidget"):
            return
        nameItem = self.plannedJointStateTableWidget.item(row, 0)
        plannedItem = self.plannedJointStateTableWidget.item(row, 2)
        if nameItem is None or plannedItem is None:
            return
        jointName = str(nameItem.text())
        text = str(plannedItem.text()).strip()
        if not text:
            self._jointControlPlannedValues.pop(jointName, None)
            return
        try:
            float(text)
        except Exception:
            plannedItem.setBackground(qt.QColor("mistyrose"))
            if hasattr(self, "jointControlStatusLabel"):
                self.jointControlStatusLabel.text = _(
                    f"Joint Control status: planned value for {jointName} is not numeric."
                )
            return
        plannedItem.setBackground(qt.QColor("white"))
        self._jointControlPlannedValues[jointName] = text

    def onSetPlannedFromCurrentButton(self, checked=False) -> None:
        self._jointControlPlannedValues = {}
        for name, rawValue in self._jointControlCurrentRawValues.items():
            self._jointControlPlannedValues[name] = f"{float(rawValue):.6g}"
        names = self._latestJointState.get("names", []) if isinstance(self._latestJointState, dict) else []
        positions = self._latestJointState.get("positions", []) if isinstance(self._latestJointState, dict) else []
        self._updateJointControlFromJointState(names, positions)

    def onClearPlannedJointValuesButton(self, checked=False) -> None:
        self._jointControlPlannedValues = {}
        if hasattr(self, "plannedJointStateTableWidget"):
            self.plannedJointStateTableWidget.blockSignals(True)
            rowCount = self.plannedJointStateTableWidget.rowCount
            if callable(rowCount):
                rowCount = rowCount()
            for row in range(int(rowCount)):
                item = self.plannedJointStateTableWidget.item(row, 2)
                if item is not None:
                    item.setText("")
                    item.setBackground(qt.QColor("white"))
            self.plannedJointStateTableWidget.blockSignals(False)
        if hasattr(self, "jointControlStatusLabel"):
            self.jointControlStatusLabel.text = _(
                "Planned/FK stage: planned values cleared. Current joint state display is unchanged."
            )

    def _currentTriggerNameText(self) -> str:
        if not hasattr(self, "triggerTopicComboBox"):
            return ""
        return str(self.triggerTopicComboBox.currentText).strip()

    def _setTriggerNameComboText(self, name: str) -> None:
        if not hasattr(self, "triggerTopicComboBox"):
            return
        name = str(name or "").strip()
        index = self.triggerTopicComboBox.findText(name) if name else -1
        self.triggerTopicComboBox.blockSignals(True)
        if index >= 0:
            self.triggerTopicComboBox.setCurrentIndex(index)
        elif self.triggerTopicComboBox.isEditable() and self.triggerTopicComboBox.lineEdit() is not None:
            self.triggerTopicComboBox.setCurrentIndex(-1)
            self.triggerTopicComboBox.lineEdit().setText(name)
        self.triggerTopicComboBox.blockSignals(False)

    def refreshTriggerNameComboBox(self, preserveCurrent: bool = True) -> None:
        """Refresh the command-enable service/topic dropdown from existing SlicerROS2 nodes.

        The list is intentionally limited to existing SlicerROS2 service clients
        or Bool publishers, depending on the selected mode. Manual text is
        preserved in the editable field and will create the client/publisher only
        when Apply Desired State is pressed.
        """
        if not hasattr(self, "triggerTopicComboBox") or self.logic is None:
            return
        currentText = self._currentTriggerNameText() if preserveCurrent else ""
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        mode = self.triggerModeComboBox.currentText if hasattr(self, "triggerModeComboBox") else "None"

        if mode == "SetBoolString service":
            names = self.logic.listSetBoolServiceClientNames(robotNode)
        elif mode == "Bool topic":
            names = self.logic.listBoolPublisherTopics(robotNode)
        else:
            names = []

        uniqueNames = []
        for name in names:
            name = str(name or "").strip()
            if name and name not in uniqueNames:
                uniqueNames.append(name)

        self.triggerTopicComboBox.blockSignals(True)
        self.triggerTopicComboBox.clear()
        for name in uniqueNames:
            self.triggerTopicComboBox.addItem(name)

        if currentText:
            index = self.triggerTopicComboBox.findText(currentText)
            if index >= 0:
                self.triggerTopicComboBox.setCurrentIndex(index)
            elif self.triggerTopicComboBox.isEditable() and self.triggerTopicComboBox.lineEdit() is not None:
                self.triggerTopicComboBox.setCurrentIndex(-1)
                self.triggerTopicComboBox.lineEdit().setText(currentText)
        elif uniqueNames:
            self.triggerTopicComboBox.setCurrentIndex(0)
        elif self.triggerTopicComboBox.isEditable() and self.triggerTopicComboBox.lineEdit() is not None:
            self.triggerTopicComboBox.setCurrentIndex(-1)
            self.triggerTopicComboBox.lineEdit().clear()
        self.triggerTopicComboBox.blockSignals(False)

    def _currentTriggerStatusTopicText(self) -> str:
        if not hasattr(self, "triggerStatusTopicComboBox"):
            return ""
        return str(self.triggerStatusTopicComboBox.currentText).strip()

    def _setTriggerStatusTopicComboText(self, topic: str) -> None:
        if not hasattr(self, "triggerStatusTopicComboBox"):
            return
        topic = str(topic or "").strip()
        index = self.triggerStatusTopicComboBox.findText(topic) if topic else -1
        self.triggerStatusTopicComboBox.blockSignals(True)
        if index >= 0:
            self.triggerStatusTopicComboBox.setCurrentIndex(index)
        elif self.triggerStatusTopicComboBox.isEditable() and self.triggerStatusTopicComboBox.lineEdit() is not None:
            self.triggerStatusTopicComboBox.setCurrentIndex(-1)
            self.triggerStatusTopicComboBox.lineEdit().setText(topic)
        self.triggerStatusTopicComboBox.blockSignals(False)

    def refreshTriggerStatusTopicComboBox(self, preserveCurrent: bool = True) -> None:
        """Refresh the command-enable status dropdown from existing Bool subscribers."""
        if not hasattr(self, "triggerStatusTopicComboBox") or self.logic is None:
            return
        currentText = self._currentTriggerStatusTopicText() if preserveCurrent else ""
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None
        topics = self.logic.listBoolSubscriberTopics(robotNode)

        uniqueTopics = []
        for topic in topics:
            topic = str(topic or "").strip()
            if topic and topic not in uniqueTopics:
                uniqueTopics.append(topic)

        self.triggerStatusTopicComboBox.blockSignals(True)
        self.triggerStatusTopicComboBox.clear()
        for topic in uniqueTopics:
            self.triggerStatusTopicComboBox.addItem(topic)

        if currentText:
            index = self.triggerStatusTopicComboBox.findText(currentText)
            if index >= 0:
                self.triggerStatusTopicComboBox.setCurrentIndex(index)
            elif self.triggerStatusTopicComboBox.isEditable() and self.triggerStatusTopicComboBox.lineEdit() is not None:
                self.triggerStatusTopicComboBox.setCurrentIndex(-1)
                self.triggerStatusTopicComboBox.lineEdit().setText(currentText)
        elif uniqueTopics:
            self.triggerStatusTopicComboBox.setCurrentIndex(0)
        elif self.triggerStatusTopicComboBox.isEditable() and self.triggerStatusTopicComboBox.lineEdit() is not None:
            self.triggerStatusTopicComboBox.setCurrentIndex(-1)
            self.triggerStatusTopicComboBox.lineEdit().clear()
        self.triggerStatusTopicComboBox.blockSignals(False)

    def _updateTriggerModeUi(self) -> None:
        if not hasattr(self, "triggerModeComboBox"):
            return
        mode = self.triggerModeComboBox.currentText
        enabled = mode != "None"
        self.triggerTopicComboBox.enabled = enabled
        self.triggerStatusTopicComboBox.enabled = enabled
        self.triggerEnableCheckBox.enabled = enabled
        if mode == "SetBoolString service":
            if hasattr(self, "triggerNameLabel"):
                self.triggerNameLabel.text = _("Enable service:")
            self.triggerTopicComboBox.lineEdit().placeholderText = "robot/bridge_enable"
            self.publishTriggerButton.text = _("Apply Desired State")
            self.triggerTopicComboBox.toolTip = _(
                "ROS2 service name. SlicerROS2 client type: SetBoolString. "
                "AIRS default: airs/bridge_enable; KIMM default: kimm/bridge_enable. "
                "A client is created only when Apply Desired State is pressed."
            )
        elif mode == "Bool topic":
            if hasattr(self, "triggerNameLabel"):
                self.triggerNameLabel.text = _("Enable topic:")
            self.triggerTopicComboBox.lineEdit().placeholderText = "/robot/command_enable"
            self.publishTriggerButton.text = _("Apply Desired State")
            self.triggerTopicComboBox.toolTip = _(
                "ROS2 topic name. Message type: std_msgs/msg/Bool. "
                "A publisher is created only when Apply Desired State is pressed."
            )
        else:
            if hasattr(self, "triggerNameLabel"):
                self.triggerNameLabel.text = _("Enable service/topic:")
            self.triggerTopicComboBox.lineEdit().placeholderText = ""
            self.publishTriggerButton.text = _("Apply Desired State")
            self.triggerStatusLabel.text = _("Command enable status: disabled for this robot configuration.")
            self._updateTriggerCurrentStatusUi(None, "No trigger configured")

    def onTriggerModeChanged(self, value=None) -> None:
        # The mode determines which existing SlicerROS2 nodes are valid for
        # the Enable service/topic combo box. When the mode changes, clear the
        # current entry so a SetBool service name is not accidentally reused as
        # a Bool topic, or vice versa. This does not create any ROS2 node.
        mode = self.triggerModeComboBox.currentText if hasattr(self, "triggerModeComboBox") else "None"
        if self._parameterNode:
            self._parameterNode.triggerMode = str(mode)
            self._parameterNode.triggerTopic = ""
        self._updateTriggerModeUi()
        self.refreshTriggerNameComboBox(preserveCurrent=False)
        self._setTriggerNameComboText("")
        self._disconnectTriggerStatusSubscriber()
        if hasattr(self, "triggerStatusLabel"):
            if mode == "SetBoolString service":
                self.triggerStatusLabel.text = _(
                    "Command enable status: mode changed to SetBool service. "
                    "The Enable service list now shows compatible SetBool service clients. "
                    "Select an existing service or type a new service name, then click Apply Desired State."
                )
            elif mode == "Bool topic":
                self.triggerStatusLabel.text = _(
                    "Command enable status: mode changed to Bool topic. "
                    "The Enable topic list now shows compatible Bool publishers. "
                    "Select an existing topic or type a new topic name, then click Apply Desired State."
                )
            else:
                self.triggerStatusLabel.text = _(
                    "Command enable status: disabled for this robot configuration."
                )
        self._updateSendCommandButtonState()

    def onTriggerTopicChanged(self, text) -> None:
        if self._parameterNode:
            self._parameterNode.triggerTopic = str(text).strip()
        self._updateSendCommandButtonState()

    def onTriggerStatusTopicChanged(self, text) -> None:
        if self._parameterNode:
            self._parameterNode.triggerStatusTopic = str(text).strip()
        normalizedText = self.logic._normalizeTopicName(str(text).strip()) if self.logic is not None and str(text).strip() else ""
        if self._triggerStatusSubscriberNode is not None and normalizedText != self._triggerStatusTopic:
            self._disconnectTriggerStatusSubscriber()
            self._updateTriggerCurrentStatusUi(None, "Not subscribed")
        if hasattr(self, "triggerStatusLabel"):
            self.triggerStatusLabel.text = _(
                "Command enable status: status topic edited. Click Subscribe to Status when the topic name is complete."
            )

    def onSubscribeTriggerStatusButton(self, checked=False) -> None:
        self._connectTriggerStatusSubscriber()

    def onTriggerEnabledChanged(self, checked) -> None:
        if self._parameterNode:
            self._parameterNode.triggerEnabled = bool(checked)

    def _disconnectTriggerStatusSubscriber(self) -> None:
        if self._triggerStatusSubscriberNode is not None and self._triggerStatusObserverTag is not None:
            try:
                self._triggerStatusSubscriberNode.RemoveObserver(self._triggerStatusObserverTag)
            except Exception:
                pass
        self._triggerStatusSubscriberNode = None
        self._triggerStatusObserverTag = None
        self._triggerStatusTopic = ""

    def _connectTriggerStatusSubscriber(self) -> None:
        if self.logic is None or not hasattr(self, "triggerStatusTopicComboBox"):
            return
        mode = self.triggerModeComboBox.currentText if hasattr(self, "triggerModeComboBox") else "None"
        statusTopic = self._currentTriggerStatusTopicText()
        robotNode = self.robotSelector.currentNode() if hasattr(self, "robotSelector") else None

        if mode == "None" or not statusTopic or robotNode is None:
            self._disconnectTriggerStatusSubscriber()
            if hasattr(self, "triggerCurrentStatusLabel"):
                reason = "No status topic configured" if mode != "None" else "No trigger configured"
                self._updateTriggerCurrentStatusUi(None, reason)
            return

        normalizedTopic = self.logic._normalizeTopicName(statusTopic)
        if (self._triggerStatusSubscriberNode is not None
                and self._triggerStatusTopic == normalizedTopic):
            return

        self._disconnectTriggerStatusSubscriber()
        try:
            subscriber, normalizedTopic, action = self.logic.getOrCreateBoolSubscriber(robotNode, statusTopic)
            self._triggerStatusSubscriberNode = subscriber
            self._triggerStatusTopic = normalizedTopic
            self._triggerStatusObserverTag = subscriber.AddObserver(vtk.vtkCommand.ModifiedEvent, self._onTriggerStatusReceived)
            self.triggerStatusLabel.text = _(f"Command enable status: listening to {normalizedTopic} ({action} subscriber).")
            self._onTriggerStatusReceived(subscriber, vtk.vtkCommand.ModifiedEvent)
        except Exception as exc:
            self._disconnectTriggerStatusSubscriber()
            self._updateTriggerCurrentStatusUi(None, "Status unavailable")
            self.triggerStatusLabel.text = _(f"Command enable status: could not subscribe to status topic: {exc}")

    def _updateTriggerCurrentStatusUi(self, status, detail: str = "") -> None:
        if not hasattr(self, "triggerCurrentStatusLabel"):
            return
        if status is True:
            self.triggerCurrentStatusLabel.text = _("ENABLED")
            self.triggerCurrentStatusLabel.setStyleSheet("background-color: lightgreen; font-weight: bold; padding: 3px;")
        elif status is False:
            self.triggerCurrentStatusLabel.text = _("DISABLED")
            self.triggerCurrentStatusLabel.setStyleSheet("background-color: lightcoral; font-weight: bold; padding: 3px;")
        else:
            self.triggerCurrentStatusLabel.text = _(detail or "Unknown")
            self.triggerCurrentStatusLabel.setStyleSheet("background-color: khaki; font-weight: normal; padding: 3px;")

    def _onTriggerStatusReceived(self, caller, event) -> None:
        try:
            message = caller.GetLastMessage()
            status = self.logic.boolFromMessage(message) if self.logic is not None else None
        except Exception:
            status = None

        self._updateTriggerCurrentStatusUi(status)
        # Do not synchronize the desired-state checkbox from status feedback.
        # The checkbox represents the next value the user wants to send when
        # pressing Apply Desired State. The status topic represents the
        # reported/current state and is displayed separately.

    def onPublishTriggerButton(self, checked=False) -> None:
        robotNode = self.robotSelector.currentNode()
        triggerMode = self.triggerModeComboBox.currentText
        triggerName = self._currentTriggerNameText()
        triggerValue = bool(self.triggerEnableCheckBox.checked)

        try:
            result = self.logic.applyTrigger(robotNode, triggerMode, triggerName, triggerValue)
            normalizedName = result.get("name", triggerName) if isinstance(result, dict) else triggerName
            action = result.get("action", "used") if isinstance(result, dict) else "used"
            mode = result.get("mode", triggerMode) if isinstance(result, dict) else triggerMode
            message = result.get("message", "") if isinstance(result, dict) else ""
            if normalizedName:
                self._setTriggerNameComboText(normalizedName)
            if mode == "None":
                self.triggerStatusLabel.text = _("Command enable status: no command-enable mechanism configured; pose command can be sent directly.")
            else:
                self.triggerStatusLabel.text = _(
                    f"Command enable status: {mode} {normalizedName} set to {triggerValue} "
                    f"({action}). {message}"
                )
        except Exception as exc:
            self.triggerStatusLabel.text = _(f"Command enable status: failed to apply command enable: {exc}")
            logging.exception("Failed to apply trigger command")

    def onSendCommandButton(self, checked=False) -> None:
        robotNode = self.robotSelector.currentNode()
        desiredTransformNode = self.desiredTransformSelector.currentNode()
        commandTopic = self._currentCommandTopicText()
        frameId = self.frameIdLineEdit.text.strip()

        try:
            result = self.logic.publishDesiredPose(robotNode, desiredTransformNode, commandTopic, frameId)
            normalizedTopic = result.get("topic", commandTopic) if isinstance(result, dict) else commandTopic
            publisherAction = result.get("publisherAction", "used") if isinstance(result, dict) else "used"
            self._setCommandTopicComboText(normalizedTopic)
            self.statusLabel.text = _(
                f"Status: PoseStamped command published on {normalizedTopic} "
                f"from transform '{desiredTransformNode.GetName()}' "
                f"({publisherAction} publisher)."
            )
        except Exception as exc:
            self.statusLabel.text = _(f"Status: failed to send command: {exc}")
            logging.exception("Failed to send robot command")


#
# CustomControlLogic
#


class CustomControlLogic(ScriptedLoadableModuleLogic):
    """Logic for custom control operations."""

    def __init__(self) -> None:
        ScriptedLoadableModuleLogic.__init__(self)

    def getParameterNode(self):
        return CustomControlParameterNode(super().getParameterNode())

    def _normalizeTopicName(self, topicName: str) -> str:
        """Return a ROS-style absolute topic name."""
        topic = (topicName or "").strip()
        if not topic:
            raise ValueError("Command topic is empty")
        if not topic.startswith("/"):
            topic = f"/{topic}"
        return topic

    def _normalizeServiceName(self, serviceName: str) -> str:
        """Return a ROS2 service name while preserving the AIRS/SlicerROS2 convention.

        The existing AIRS Slicer module uses ``airs/bridge_enable`` without a
        leading slash for SlicerROS2 service clients. Topics are normalized to
        leading slash elsewhere, but service names are kept as typed except that
        a leading slash is removed for consistency with AIRS.
        """
        service = (serviceName or "").strip()
        if not service:
            raise ValueError("Trigger service name is empty")
        return service[1:] if service.startswith("/") else service

    def _getROS2NodeForRobot(self, robotNode):
        """Return the ROS2 node associated with the selected robot, or the default ROS2 node."""
        if robotNode is not None and robotNode.GetNodeReference("node"):
            return robotNode.GetNodeReference("node")

        rosLogic = slicer.util.getModuleLogic("ROS2")
        if rosLogic and hasattr(rosLogic, "GetDefaultROS2Node"):
            return rosLogic.GetDefaultROS2Node()

        return slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLROS2NodeNode")

    def _nodeName(self, node) -> str:
        if node is None:
            return ""
        try:
            return str(node.GetName() or "")
        except Exception:
            return ""

    def _findRobotLinkTransform(self, robotNode, linkName: str):
        """Find the current Slicer transform node for a robot link name.

        This mirrors the naming convention used by SlicerROS2 robot models, where
        mesh nodes are typically named <link>_model_<index> and parented under
        the live link transform. It also falls back to TF lookup nodes for links
        without visible mesh nodes.
        """
        if robotNode is None or not linkName:
            return None

        linkName = str(linkName)
        scene = slicer.mrmlScene
        if scene is None:
            return None

        # First search model nodes referenced by the selected robot. This avoids
        # accidentally picking a different robot's transform if several robots are loaded.
        candidates = []
        try:
            numberOfModels = robotNode.GetNumberOfNodeReferences("model")
        except Exception:
            numberOfModels = 0
        prefix = f"{linkName}_model_"
        for index in range(numberOfModels):
            try:
                modelNode = robotNode.GetNthNodeReference("model", index)
            except Exception:
                modelNode = None
            if modelNode is None:
                continue
            modelName = self._nodeName(modelNode)
            if modelName == linkName or modelName.startswith(prefix):
                parentTransform = modelNode.GetParentTransformNode()
                if parentTransform is not None:
                    candidates.append((modelName, parentTransform))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]

        # Fallback: scan all model nodes using the same SlicerROS2 naming convention.
        candidates = []
        for index in range(scene.GetNumberOfNodesByClass("vtkMRMLModelNode")):
            modelNode = scene.GetNthNodeByClass(index, "vtkMRMLModelNode")
            if modelNode is None:
                continue
            modelName = self._nodeName(modelNode)
            if modelName == linkName or modelName.startswith(prefix):
                parentTransform = modelNode.GetParentTransformNode()
                if parentTransform is not None:
                    candidates.append((modelName, parentTransform))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]

        # Fallback for links that have no mesh model but do have a TF lookup node.
        for index in range(scene.GetNumberOfNodesByClass("vtkMRMLROS2Tf2LookupNode")):
            lookupNode = scene.GetNthNodeByClass(index, "vtkMRMLROS2Tf2LookupNode")
            if lookupNode is None:
                continue
            try:
                childID = lookupNode.GetChildID() or ""
            except Exception:
                childID = ""
            if childID == linkName or childID.endswith(f"/{linkName}"):
                return lookupNode

        return None

    def getCurrentRobotTipWorldMatrix(self, robotNode):
        """Return the current robot tip/platform pose as a vtkMatrix4x4 in world coordinates.

        For Stewart-platform-style robots, this is typically the distal/platform
        link returned by robotNode.FindRootAndTipLinks(). The returned matrix can
        be used to initialize a user-created desired-pose transform.
        """
        if robotNode is None:
            return None

        tipLink = ""
        try:
            rootAndTip = robotNode.FindRootAndTipLinks()
            if rootAndTip and len(rootAndTip) >= 2:
                tipLink = rootAndTip[1]
        except Exception:
            tipLink = ""

        candidateLinks = []
        if tipLink:
            candidateLinks.append(tipLink)

        # Common names for platform/distal links, used only as fallbacks.
        for fallbackLink in ("distal_ring", "platform", "moving_platform", "tool0", "ee_link", "end_effector"):
            if fallbackLink not in candidateLinks:
                candidateLinks.append(fallbackLink)

        for linkName in candidateLinks:
            transformNode = self._findRobotLinkTransform(robotNode, linkName)
            if transformNode is None:
                continue
            matrix = vtk.vtkMatrix4x4()
            transformNode.GetMatrixTransformToWorld(matrix)
            return matrix

        return None

    def _publisherTopic(self, publisher) -> str:
        """Return the topic name for a SlicerROS2 publisher node if available."""
        if publisher is None:
            return ""
        for getterName in ("GetTopic", "GetTopicName"):
            getter = getattr(publisher, getterName, None)
            if getter is not None:
                try:
                    return str(getter())
                except Exception:
                    pass
        return ""

    def _iterScenePublisherNodes(self):
        """Yield MRML nodes that look like SlicerROS2 publisher nodes.

        This intentionally mirrors what the SlicerROS2 Topics tab manages: existing
        publisher MRML nodes in the Slicer scene. It does not query the external ROS
        graph for arbitrary topics.
        """
        scene = slicer.mrmlScene
        if scene is None:
            return
        for index in range(scene.GetNumberOfNodes()):
            node = scene.GetNthNode(index)
            if node is None:
                continue
            className = self._nodeClassName(node)
            if "Publisher" not in className:
                continue
            if hasattr(node, "Publish") and (hasattr(node, "GetTopic") or hasattr(node, "GetTopicName")):
                yield node

    def _iterROS2NodePublisherNodes(self, ros2Node):
        """Yield publisher nodes owned by a specific SlicerROS2 node, if its API exposes them.

        Different SlicerROS2 builds expose different helper methods. The scene scan
        above is the primary path; this method is a best-effort supplement.
        """
        if ros2Node is None:
            return

        # Count/index style APIs. These may or may not exist in a given build.
        countGetters = (
            "GetNumberOfPublisherNodes",
            "GetNumberOfPublishers",
            "GetNumberOfROS2PublisherNodes",
        )
        nodeGetters = (
            "GetNthPublisherNode",
            "GetPublisherNode",
            "GetNthROS2PublisherNode",
        )
        for countName in countGetters:
            countGetter = getattr(ros2Node, countName, None)
            if countGetter is None:
                continue
            try:
                count = int(countGetter())
            except Exception:
                continue
            for nodeGetterName in nodeGetters:
                nodeGetter = getattr(ros2Node, nodeGetterName, None)
                if nodeGetter is None:
                    continue
                for i in range(count):
                    try:
                        node = nodeGetter(i)
                    except Exception:
                        continue
                    if node is not None:
                        yield node
                return

    def listPoseStampedPublisherTopics(self, robotNode=None) -> list[str]:
        """List existing SlicerROS2 publisher topics compatible with PoseStamped.

        The dropdown should reflect publishers already present in the SlicerROS2
        Topics tab. It should not list arbitrary ROS graph subscribers or topics
        unless a SlicerROS2 publisher node already exists for them. The combo box
        remains editable, so the user can type a new topic and the module will
        create the publisher at send time.
        """
        ros2Node = self._getROS2NodeForRobot(robotNode)
        topics = []

        seenNodeIds = set()
        for source in (self._iterROS2NodePublisherNodes(ros2Node), self._iterScenePublisherNodes()):
            for publisher in source or []:
                if publisher is None:
                    continue
                try:
                    nodeId = publisher.GetID() if hasattr(publisher, "GetID") else str(id(publisher))
                except Exception:
                    nodeId = str(id(publisher))
                if nodeId in seenNodeIds:
                    continue
                seenNodeIds.add(nodeId)

                try:
                    if not self._publisherReturnsPoseStampedMessage(publisher):
                        continue
                    topic = self._publisherTopic(publisher)
                except Exception:
                    continue

                topic = str(topic or "").strip()
                if topic and topic not in topics:
                    topics.append(topic)

        topics.sort()
        return topics

    def listPoseStampedGraphTopics(self, robotNode=None) -> list[str]:
        """Backward-compatible alias: now returns SlicerROS2 publisher topics only."""
        return self.listPoseStampedPublisherTopics(robotNode)

    def _nodeClassName(self, node) -> str:
        if node is None:
            return "None"
        getClassName = getattr(node, "GetClassName", None)
        if getClassName is not None:
            try:
                return str(getClassName())
            except Exception:
                pass
        return type(node).__name__

    def _registeredPublisherNodes(self, ros2Node) -> str:
        try:
            return str(ros2Node.RegisteredROS2PublisherNodes())
        except Exception as exc:
            return f"Could not query RegisteredROS2PublisherNodes(): {exc}"

    def _serviceClientServiceName(self, client) -> str:
        """Return the service name for a SlicerROS2 service-client node if available."""
        if client is None:
            return ""
        for getterName in ("GetService", "GetServiceName"):
            getter = getattr(client, getterName, None)
            if getter is not None:
                try:
                    return str(getter())
                except Exception:
                    pass
        return ""

    def _iterSceneServiceClientNodes(self):
        """Yield MRML nodes that look like SlicerROS2 service-client nodes."""
        scene = slicer.mrmlScene
        if scene is None:
            return
        for index in range(scene.GetNumberOfNodes()):
            node = scene.GetNthNode(index)
            if node is None:
                continue
            className = self._nodeClassName(node)
            if "ServiceClient" not in className:
                continue
            if hasattr(node, "CreateBlankRequest") and hasattr(node, "SendAsyncRequest") and (hasattr(node, "GetService") or hasattr(node, "GetServiceName")):
                yield node

    def _iterROS2NodeServiceClientNodes(self, ros2Node):
        """Yield service-client nodes owned by a SlicerROS2 node, if exposed by its API."""
        if ros2Node is None:
            return
        countGetters = (
            "GetNumberOfServiceClientNodes",
            "GetNumberOfServiceClients",
            "GetNumberOfROS2ServiceClientNodes",
        )
        nodeGetters = (
            "GetNthServiceClientNode",
            "GetServiceClientNode",
            "GetNthROS2ServiceClientNode",
        )
        for countName in countGetters:
            countGetter = getattr(ros2Node, countName, None)
            if countGetter is None:
                continue
            try:
                count = int(countGetter())
            except Exception:
                continue
            for nodeGetterName in nodeGetters:
                nodeGetter = getattr(ros2Node, nodeGetterName, None)
                if nodeGetter is None:
                    continue
                for i in range(count):
                    try:
                        node = nodeGetter(i)
                    except Exception:
                        continue
                    if node is not None:
                        yield node
                return

    def listSetBoolServiceClientNames(self, robotNode=None) -> list[str]:
        """List existing SlicerROS2 SetBool-compatible service-client names."""
        ros2Node = self._getROS2NodeForRobot(robotNode)
        services = []
        seenNodeIds = set()
        for source in (self._iterROS2NodeServiceClientNodes(ros2Node), self._iterSceneServiceClientNodes()):
            for client in source or []:
                if client is None:
                    continue
                try:
                    nodeId = client.GetID() if hasattr(client, "GetID") else str(id(client))
                except Exception:
                    nodeId = str(id(client))
                if nodeId in seenNodeIds:
                    continue
                seenNodeIds.add(nodeId)
                try:
                    if not self._serviceClientLooksLikeSetBool(client):
                        continue
                    service = self._serviceClientServiceName(client)
                except Exception:
                    continue
                service = str(service or "").strip()
                if service and service not in services:
                    services.append(service)
        services.sort()
        return services

    def listBoolPublisherTopics(self, robotNode=None) -> list[str]:
        """List existing SlicerROS2 Bool publisher topics."""
        ros2Node = self._getROS2NodeForRobot(robotNode)
        topics = []
        seenNodeIds = set()
        for source in (self._iterROS2NodePublisherNodes(ros2Node), self._iterScenePublisherNodes()):
            for publisher in source or []:
                if publisher is None:
                    continue
                try:
                    nodeId = publisher.GetID() if hasattr(publisher, "GetID") else str(id(publisher))
                except Exception:
                    nodeId = str(id(publisher))
                if nodeId in seenNodeIds:
                    continue
                seenNodeIds.add(nodeId)
                try:
                    if not self._publisherLooksLikeBool(publisher):
                        continue
                    topic = self._publisherTopic(publisher)
                except Exception:
                    continue
                topic = str(topic or "").strip()
                if topic and topic not in topics:
                    topics.append(topic)
        topics.sort()
        return topics

    def listBoolSubscriberTopics(self, robotNode=None) -> list[str]:
        """List existing SlicerROS2 Bool subscriber topics."""
        ros2Node = self._getROS2NodeForRobot(robotNode)
        topics = []
        seenNodeIds = set()
        for source in (self._iterROS2NodeSubscriberNodes(ros2Node), self._iterSceneSubscriberNodes()):
            for subscriber in source or []:
                if subscriber is None:
                    continue
                try:
                    nodeId = subscriber.GetID() if hasattr(subscriber, "GetID") else str(id(subscriber))
                except Exception:
                    nodeId = str(id(subscriber))
                if nodeId in seenNodeIds:
                    continue
                seenNodeIds.add(nodeId)
                try:
                    if not self._subscriberLooksLikeBool(subscriber):
                        continue
                    topic = self._subscriberTopic(subscriber)
                except Exception:
                    continue
                topic = str(topic or "").strip()
                if topic and topic not in topics:
                    topics.append(topic)
        topics.sort()
        return topics

    def _registeredServiceClientNodes(self, ros2Node) -> str:
        try:
            return str(ros2Node.RegisteredROS2ServiceClientNodes())
        except Exception as exc:
            return f"Could not query RegisteredROS2ServiceClientNodes(): {exc}"

    def _getExistingServiceClientByService(self, ros2Node, serviceName: str):
        """Return an existing service client for serviceName if this SlicerROS2 build exposes one."""
        for getterName in ("GetServiceClientNodeByService", "GetServiceClientNodeByServiceName"):
            getter = getattr(ros2Node, getterName, None)
            if getter is not None:
                try:
                    client = getter(serviceName)
                    if client is not None:
                        return client
                except Exception:
                    pass

        # Fallback: scan the MRML scene for ROS2 service client nodes whose GetService() matches.
        scene = slicer.mrmlScene
        if scene is None:
            return None
        for className in ("vtkMRMLROS2ServiceClientNode", "vtkMRMLNode"):
            try:
                nodes = slicer.util.getNodesByClass(className)
            except Exception:
                nodes = []
            for node in nodes:
                if not hasattr(node, "GetService"):
                    continue
                try:
                    if node.GetService() == serviceName:
                        return node
                except Exception:
                    continue
        return None

    def _serviceClientLooksLikeSetBool(self, client) -> bool:
        if client is None or not hasattr(client, "CreateBlankRequest") or not hasattr(client, "SendAsyncRequest"):
            return False
        try:
            req = client.CreateBlankRequest()
        except Exception:
            return False
        return hasattr(req, "SetData") or hasattr(req, "SetValue") or hasattr(req, "SetRequest")

    def _setBoolStringServiceClientCandidates(self, ros2Node):
        registered = self._registeredServiceClientNodes(ros2Node)
        # The existing AIRS Slicer module uses SlicerROS2 service type
        # "SetBoolString" for the bridge-enable service.
        possible = [
            "SetBoolString",
            "vtkMRMLROS2ServiceClientSetBoolStringNode",
            # Keep intuitive names as fallbacks only for future builds that may register them.
            "SetBool",
            "vtkMRMLROS2ServiceClientSetBoolNode",
            "vtkMRMLROSServiceClientSetBoolNode",
        ]
        ordered = []
        for name in possible:
            if name in registered and name not in ordered:
                ordered.append(name)
        for name in possible:
            if name not in ordered:
                ordered.append(name)
        return ordered, registered

    def _getOrCreateSetBoolStringServiceClient(self, ros2Node, serviceName: str):
        if ros2Node is None:
            raise ValueError("No ROS2 node is available for the selected robot")
        service = self._normalizeServiceName(serviceName)

        existingClient = self._getExistingServiceClientByService(ros2Node, service)
        if existingClient is not None:
            if self._serviceClientLooksLikeSetBool(existingClient):
                return existingClient, service, "reused"
            raise RuntimeError(
                f"A service client already exists for '{service}', but it does not look like "
                "a SlicerROS2 SetBoolString client. Remove it in the ROS2 module or choose a new service. "
                f"Client class: {self._nodeClassName(existingClient)}. "
                f"Registered service clients: {self._registeredServiceClientNodes(ros2Node)}"
            )

        clientTypeCandidates, registered = self._setBoolStringServiceClientCandidates(ros2Node)
        errors = []
        for clientType in clientTypeCandidates:
            client = None
            try:
                client = ros2Node.CreateAndAddServiceClientNode(clientType, service)
            except Exception as exc:
                errors.append(f"{clientType}: create failed: {exc}")
                continue

            if client is None:
                errors.append(f"{clientType}: CreateAndAddServiceClientNode returned None")
                continue

            if self._serviceClientLooksLikeSetBool(client):
                return client, service, "created"

            errors.append(f"{clientType}: created {self._nodeClassName(client)}, but request did not look like SetBoolString")
            try:
                ros2Node.RemoveAndDeleteServiceClientNode(service)
            except Exception:
                pass

        raise RuntimeError(
            f"Could not create a compatible SetBoolString service client for '{service}'. "
            f"In this SlicerROS2 build, std_srvs/srv/SetBool is expected to be registered as SetBoolString. "
            f"Tried: {', '.join(clientTypeCandidates)}. "
            f"Registered service clients: {registered}. Errors: {'; '.join(errors)}"
        )

    def callSetBoolStringTrigger(self, robotNode, serviceName: str, triggerValue: bool, timeoutSec: float = 2.0) -> dict:
        """Call the SlicerROS2 SetBoolString service client used by AIRS.

        The AIRS Slicer module creates this with
        CreateAndAddServiceClientNode("SetBoolString", "airs/bridge_enable"),
        then sends a blank request with req.SetValue(bool).
        """
        if robotNode is None:
            raise ValueError("No robot node selected")

        ros2Node = self._getROS2NodeForRobot(robotNode)
        client, normalizedService, clientAction = self._getOrCreateSetBoolStringServiceClient(ros2Node, serviceName)

        req = client.CreateBlankRequest()
        value = bool(triggerValue)
        if hasattr(req, "SetValue"):
            req.SetValue(value)
        elif hasattr(req, "SetData"):
            req.SetData(value)
        elif hasattr(req, "SetRequest"):
            req.SetRequest(value)
        else:
            raise RuntimeError("SetBoolString request object has no recognized boolean setter")

        client.SendAsyncRequest(req)
        rosLogic = slicer.util.getModuleLogic("ROS2")
        if rosLogic is not None and hasattr(rosLogic, "WaitForServiceResponse"):
            if not rosLogic.WaitForServiceResponse(client, float(timeoutSec)):
                raise TimeoutError(f"Timed out waiting for response from {normalizedService}")
            response = client.GetLastResponse()
        else:
            # Without the helper, the request is still sent, but we cannot reliably
            # wait for the response from this module.
            response = None

        responseMessage = ""
        responseSuccess = None
        if response is not None:
            for getterName in ("GetMessage", "GetMessage_", "GetStatusMessage"):
                getter = getattr(response, getterName, None)
                if getter is not None:
                    try:
                        responseMessage = str(getter())
                        break
                    except Exception:
                        pass
            for getterName in ("GetSuccess", "GetResult", "GetOk"):
                getter = getattr(response, getterName, None)
                if getter is not None:
                    try:
                        responseSuccess = bool(getter())
                        break
                    except Exception:
                        pass

        logging.info(
            "SetBoolString trigger service called: robot='%s', service='%s', value=%s, client=%s, response_success=%s, response_message='%s'",
            robotNode.GetName(), normalizedService, value, clientAction, responseSuccess, responseMessage
        )

        return {
            "client": client,
            "name": normalizedService,
            "mode": "SetBoolString service",
            "action": clientAction,
            "value": value,
            "responseSuccess": responseSuccess,
            "message": responseMessage,
        }

    def applyTrigger(self, robotNode, triggerMode: str, triggerName: str, triggerValue: bool) -> dict:
        """Apply trigger using the configured trigger mode."""
        mode = (triggerMode or "None").strip()
        if mode == "None":
            return {"mode": "None", "name": "", "action": "not used", "value": bool(triggerValue), "message": ""}
        if mode == "Bool topic":
            result = self.publishBoolTrigger(robotNode, triggerName, triggerValue)
            result["mode"] = "Bool topic"
            result["name"] = result.get("topic", triggerName)
            result["action"] = result.get("publisherAction", "used")
            return result
        if mode == "SetBoolString service":
            return self.callSetBoolStringTrigger(robotNode, triggerName, triggerValue)
        raise ValueError(f"Unsupported trigger mode: {triggerMode}")

    def _blankMessageDescription(self, publisher) -> str:
        if publisher is None:
            return "None"
        if not hasattr(publisher, "GetBlankMessage"):
            return "publisher has no GetBlankMessage()"
        try:
            blank = publisher.GetBlankMessage()
        except Exception as exc:
            return f"GetBlankMessage() failed: {exc}"
        if blank is None:
            return "GetBlankMessage() returned None"
        getClassName = getattr(blank, "GetClassName", None)
        if getClassName is not None:
            try:
                return str(getClassName())
            except Exception:
                pass
        return type(blank).__name__

    def _publisherReturnsPoseStampedMessage(self, publisher) -> bool:
        """Return True if publisher.GetBlankMessage() looks like a PoseStamped wrapper.

        In SlicerROS2, PoseStamped.GetBlankMessage() returns a message object
        with GetHeader() and SetPose(vtkMatrix4x4), not a raw vtkMatrix4x4.
        Some older/alternate builds may expose a raw matrix, so keep that as a
        permissive fallback.
        """
        if publisher is None or not hasattr(publisher, "GetBlankMessage"):
            return False
        try:
            blank = publisher.GetBlankMessage()
        except Exception:
            return False
        if isinstance(blank, vtk.vtkMatrix4x4):
            return True
        return hasattr(blank, "GetHeader") and hasattr(blank, "SetPose")

    def _removePublisherByTopicQuietly(self, ros2Node, topic: str) -> None:
        try:
            ros2Node.RemoveAndDeletePublisherNode(topic)
        except Exception:
            pass

    def _poseStampedPublisherCandidates(self, ros2Node):
        """Return PoseStamped publisher names to try, preferring names registered in this build."""
        registered = self._registeredPublisherNodes(ros2Node)
        possible = [
            # Short name used in SlicerROS2 Python examples.
            "PoseStamped",
            # Current/generated naming style used by most SlicerROS2 nodes.
            "vtkMRMLROS2PublisherPoseStampedNode",
            # Alternate/documentation naming style.
            "vtkMRMLROSPublisherPoseStampedNode",
        ]

        # Try registered full names first. RegisteredROS2PublisherNodes() returns
        # a long string, so substring matching is the most robust cross-version
        # option from Python. Do not fall back to "Pose", because that publishes
        # geometry_msgs/msg/Pose, not PoseStamped.
        ordered = []
        for name in possible:
            if name in registered and name not in ordered:
                ordered.append(name)
        for name in possible:
            if name not in ordered:
                ordered.append(name)
        return ordered, registered

    def _getOrCreatePoseStampedPublisher(self, ros2Node, commandTopic: str):
        """Reuse a compatible publisher for ``commandTopic`` or create a PoseStamped publisher."""
        if ros2Node is None:
            raise ValueError("No ROS2 node is available for the selected robot")

        topic = self._normalizeTopicName(commandTopic)

        existingPublisher = ros2Node.GetPublisherNodeByTopic(topic)
        if existingPublisher is not None:
            # Reuse only if the existing publisher looks like a PoseStamped publisher.
            # Reusing an incompatible publisher would fail later and hide the real problem.
            if self._publisherReturnsPoseStampedMessage(existingPublisher):
                return existingPublisher, topic, "reused"
            raise RuntimeError(
                f"A publisher already exists on topic '{topic}', but it does not look like "
                "a SlicerROS2 PoseStamped publisher. Expected GetBlankMessage() to return "
                "an object with GetHeader() and SetPose(vtkMatrix4x4). "
                f"Publisher class: {self._nodeClassName(existingPublisher)}. "
                f"Blank message: {self._blankMessageDescription(existingPublisher)}. "
                "Remove this publisher in the ROS2 module or choose a new topic, then try again. "
                f"Registered publisher nodes: {self._registeredPublisherNodes(ros2Node)}"
            )

        publisherTypeCandidates, registered = self._poseStampedPublisherCandidates(ros2Node)
        errors = []
        for publisherType in publisherTypeCandidates:
            publisher = None
            try:
                publisher = ros2Node.CreateAndAddPublisherNode(publisherType, topic)
            except Exception as exc:
                errors.append(f"{publisherType}: create failed: {exc}")
                continue

            if publisher is None:
                errors.append(f"{publisherType}: CreateAndAddPublisherNode returned None")
                continue

            if self._publisherReturnsPoseStampedMessage(publisher):
                return publisher, topic, "created"

            errors.append(
                f"{publisherType}: created {self._nodeClassName(publisher)}, "
                f"but blank message is {self._blankMessageDescription(publisher)}"
            )
            self._removePublisherByTopicQuietly(ros2Node, topic)

        raise RuntimeError(
            "Could not create a compatible PoseStamped publisher for topic "
            f"'{topic}'. Tried: {', '.join(publisherTypeCandidates)}. "
            "A compatible SlicerROS2 PoseStamped publisher should return a message "
            "object with GetHeader() and SetPose(vtkMatrix4x4) from GetBlankMessage(). "
            f"Registered publisher nodes: {registered}. Errors: {'; '.join(errors)}"
        )

    def _subscriberTopic(self, subscriber) -> str:
        """Return the topic name for a SlicerROS2 subscriber node if available."""
        if subscriber is None:
            return ""
        for getterName in ("GetTopic", "GetTopicName"):
            getter = getattr(subscriber, getterName, None)
            if getter is not None:
                try:
                    return str(getter())
                except Exception:
                    pass
        return ""

    def _iterSceneSubscriberNodes(self):
        """Yield MRML nodes that look like SlicerROS2 subscriber nodes."""
        scene = slicer.mrmlScene
        if scene is None:
            return
        for index in range(scene.GetNumberOfNodes()):
            node = scene.GetNthNode(index)
            if node is None:
                continue
            className = self._nodeClassName(node)
            if "Subscriber" not in className:
                continue
            if hasattr(node, "GetLastMessage") and (hasattr(node, "GetTopic") or hasattr(node, "GetTopicName")):
                yield node

    def _iterROS2NodeSubscriberNodes(self, ros2Node):
        """Yield subscriber nodes owned by a specific SlicerROS2 node, if its API exposes them."""
        if ros2Node is None:
            return
        countGetters = (
            "GetNumberOfSubscriberNodes",
            "GetNumberOfSubscribers",
            "GetNumberOfROS2SubscriberNodes",
        )
        nodeGetters = (
            "GetNthSubscriberNode",
            "GetSubscriberNode",
            "GetNthROS2SubscriberNode",
        )
        for countName in countGetters:
            countGetter = getattr(ros2Node, countName, None)
            if countGetter is None:
                continue
            try:
                count = int(countGetter())
            except Exception:
                continue
            for nodeGetterName in nodeGetters:
                nodeGetter = getattr(ros2Node, nodeGetterName, None)
                if nodeGetter is None:
                    continue
                for i in range(count):
                    try:
                        node = nodeGetter(i)
                    except Exception:
                        continue
                    if node is not None:
                        yield node
                return

    def jointStateNamesAndPositions(self, message):
        """Extract name[] and position[] from a SlicerROS2 JointState wrapper."""
        if message is None:
            return [], []
        names = []
        positions = []
        for getterName in ("GetName", "GetNames"):
            getter = getattr(message, getterName, None)
            if getter is not None:
                try:
                    names = list(getter())
                    break
                except Exception:
                    pass
        for getterName in ("GetPosition", "GetPositions"):
            getter = getattr(message, getterName, None)
            if getter is not None:
                try:
                    positions = list(getter())
                    break
                except Exception:
                    pass
        if not names and hasattr(message, "name"):
            try:
                names = list(message.name)
            except Exception:
                pass
        if not positions and hasattr(message, "position"):
            try:
                positions = list(message.position)
            except Exception:
                pass
        return names, positions

    def _subscriberLooksLikeJointState(self, subscriber) -> bool:
        """Return True if a SlicerROS2 subscriber appears to receive sensor_msgs/msg/JointState."""
        if subscriber is None or not hasattr(subscriber, "GetLastMessage"):
            return False
        className = self._nodeClassName(subscriber)
        if "JointState" in className:
            return True
        try:
            msg = subscriber.GetLastMessage()
        except Exception:
            return True
        if msg is None:
            # Accept unknown subscribers only if the class name strongly suggests JointState.
            return "Joint" in className
        names, positions = self.jointStateNamesAndPositions(msg)
        return bool(names) or hasattr(msg, "GetName") or hasattr(msg, "GetPosition")

    def _jointStateSubscriberCandidates(self, ros2Node):
        try:
            registered = str(ros2Node.RegisteredROS2SubscriberNodes())
        except Exception as exc:
            registered = f"Could not query RegisteredROS2SubscriberNodes(): {exc}"
        possible = [
            "JointState",
            "vtkMRMLROS2SubscriberJointStateNode",
            "vtkMRMLROSSubscriberJointStateNode",
        ]
        ordered = []
        for name in possible:
            if name in registered and name not in ordered:
                ordered.append(name)
        for name in possible:
            if name not in ordered:
                ordered.append(name)
        return ordered, registered

    def getOrCreateJointStateSubscriber(self, robotNode, jointStateTopic: str):
        """Reuse or create a SlicerROS2 JointState subscriber."""
        if robotNode is None:
            raise ValueError("No robot node selected")
        ros2Node = self._getROS2NodeForRobot(robotNode)
        if ros2Node is None:
            raise ValueError("No ROS2 node is available for the selected robot")

        topic = self._normalizeTopicName(jointStateTopic)
        existingSubscriber = None
        getter = getattr(ros2Node, "GetSubscriberNodeByTopic", None)
        if getter is not None:
            try:
                existingSubscriber = getter(topic)
            except Exception:
                existingSubscriber = None
        if existingSubscriber is not None:
            if self._subscriberLooksLikeJointState(existingSubscriber):
                return existingSubscriber, topic, "reused"
            raise RuntimeError(
                f"A subscriber already exists on '{topic}', but it does not look like a JointState subscriber. "
                f"Subscriber class: {self._nodeClassName(existingSubscriber)}"
            )

        subscriberTypeCandidates, registered = self._jointStateSubscriberCandidates(ros2Node)
        errors = []
        for subscriberType in subscriberTypeCandidates:
            try:
                subscriber = ros2Node.CreateAndAddSubscriberNode(subscriberType, topic)
            except Exception as exc:
                errors.append(f"{subscriberType}: create failed: {exc}")
                continue
            if subscriber is None:
                errors.append(f"{subscriberType}: CreateAndAddSubscriberNode returned None")
                continue
            if self._subscriberLooksLikeJointState(subscriber):
                return subscriber, topic, "created"
            errors.append(f"{subscriberType}: created {self._nodeClassName(subscriber)}, but it did not look like JointState")
            try:
                ros2Node.RemoveAndDeleteSubscriberNode(topic)
            except Exception:
                pass

        raise RuntimeError(
            f"Could not create a compatible JointState subscriber for '{topic}'. "
            f"Tried: {', '.join(subscriberTypeCandidates)}. Registered subscribers: {registered}. "
            f"Errors: {'; '.join(errors)}"
        )

    def listJointStateSubscriberTopics(self, robotNode=None) -> list[str]:
        """List existing SlicerROS2 JointState subscriber topics."""
        ros2Node = self._getROS2NodeForRobot(robotNode)
        topics = []
        seenNodeIds = set()
        for source in (self._iterROS2NodeSubscriberNodes(ros2Node), self._iterSceneSubscriberNodes()):
            for subscriber in source or []:
                if subscriber is None:
                    continue
                try:
                    nodeId = subscriber.GetID() if hasattr(subscriber, "GetID") else str(id(subscriber))
                except Exception:
                    nodeId = str(id(subscriber))
                if nodeId in seenNodeIds:
                    continue
                seenNodeIds.add(nodeId)
                try:
                    if not self._subscriberLooksLikeJointState(subscriber):
                        continue
                    topic = self._subscriberTopic(subscriber)
                except Exception:
                    continue
                topic = str(topic or "").strip()
                if topic and topic not in topics:
                    topics.append(topic)
        topics.sort()
        return topics

    def _subscriberLooksLikeBool(self, subscriber) -> bool:
        """Return True if a SlicerROS2 subscriber appears to receive std_msgs/msg/Bool."""
        if subscriber is None:
            return False
        if not hasattr(subscriber, "GetLastMessage"):
            return False
        # If no message has arrived yet, we cannot inspect it, so accept the
        # subscriber class/name as long as it looks like a Bool subscriber.
        className = self._nodeClassName(subscriber)
        if "Bool" in className:
            return True
        try:
            msg = subscriber.GetLastMessage()
        except Exception:
            return True
        if msg is None:
            return True
        return self.boolFromMessage(msg) is not None

    def _boolSubscriberCandidates(self, ros2Node):
        try:
            registered = str(ros2Node.RegisteredROS2SubscriberNodes())
        except Exception as exc:
            registered = f"Could not query RegisteredROS2SubscriberNodes(): {exc}"
        possible = [
            "Bool",
            "vtkMRMLROS2SubscriberBoolNode",
            "vtkMRMLROSSubscriberBoolNode",
        ]
        ordered = []
        for name in possible:
            if name in registered and name not in ordered:
                ordered.append(name)
        for name in possible:
            if name not in ordered:
                ordered.append(name)
        return ordered, registered

    def getOrCreateBoolSubscriber(self, robotNode, statusTopic: str):
        """Reuse or create a SlicerROS2 Bool subscriber for a trigger-status topic."""
        if robotNode is None:
            raise ValueError("No robot node selected")
        ros2Node = self._getROS2NodeForRobot(robotNode)
        if ros2Node is None:
            raise ValueError("No ROS2 node is available for the selected robot")

        topic = self._normalizeTopicName(statusTopic)
        existingSubscriber = None
        getter = getattr(ros2Node, "GetSubscriberNodeByTopic", None)
        if getter is not None:
            try:
                existingSubscriber = getter(topic)
            except Exception:
                existingSubscriber = None
        if existingSubscriber is not None:
            if self._subscriberLooksLikeBool(existingSubscriber):
                return existingSubscriber, topic, "reused"
            raise RuntimeError(
                f"A subscriber already exists on '{topic}', but it does not look like a Bool subscriber. "
                f"Subscriber class: {self._nodeClassName(existingSubscriber)}"
            )

        subscriberTypeCandidates, registered = self._boolSubscriberCandidates(ros2Node)
        errors = []
        for subscriberType in subscriberTypeCandidates:
            try:
                subscriber = ros2Node.CreateAndAddSubscriberNode(subscriberType, topic)
            except Exception as exc:
                errors.append(f"{subscriberType}: create failed: {exc}")
                continue
            if subscriber is None:
                errors.append(f"{subscriberType}: CreateAndAddSubscriberNode returned None")
                continue
            if self._subscriberLooksLikeBool(subscriber):
                return subscriber, topic, "created"
            errors.append(f"{subscriberType}: created {self._nodeClassName(subscriber)}, but it did not look like Bool")
            try:
                ros2Node.RemoveAndDeleteSubscriberNode(topic)
            except Exception:
                pass

        raise RuntimeError(
            f"Could not create a compatible Bool subscriber for '{topic}'. "
            f"Tried: {', '.join(subscriberTypeCandidates)}. Registered subscribers: {registered}. "
            f"Errors: {'; '.join(errors)}"
        )

    def boolFromMessage(self, message):
        """Extract a Python bool from a SlicerROS2 Bool message or wrapper."""
        if message is None:
            return None
        if isinstance(message, bool):
            return bool(message)
        for getterName in ("GetData", "GetValue", "GetBool", "GetRequest"):
            getter = getattr(message, getterName, None)
            if getter is not None:
                try:
                    return bool(getter())
                except Exception:
                    pass
        if hasattr(message, "data"):
            try:
                return bool(message.data)
            except Exception:
                pass
        return None

    def _publisherLooksLikeBool(self, publisher) -> bool:
        """Return True if the publisher can publish a bool-like message."""
        if publisher is None:
            return False
        # In SlicerROS2, Bool is documented as using Python bool as the Slicer-side type.
        # Some builds may still expose a generated blank message with SetData.
        if not hasattr(publisher, "Publish"):
            return False
        if not hasattr(publisher, "GetBlankMessage"):
            return True
        try:
            blank = publisher.GetBlankMessage()
        except Exception:
            return True
        return isinstance(blank, bool) or hasattr(blank, "SetData") or hasattr(blank, "SetValue")

    def _boolPublisherCandidates(self, ros2Node):
        registered = self._registeredPublisherNodes(ros2Node)
        possible = [
            "Bool",
            "vtkMRMLROS2PublisherBoolNode",
            "vtkMRMLROSPublisherBoolNode",
        ]
        ordered = []
        for name in possible:
            if name in registered and name not in ordered:
                ordered.append(name)
        for name in possible:
            if name not in ordered:
                ordered.append(name)
        return ordered, registered

    def _getOrCreateBoolPublisher(self, ros2Node, triggerTopic: str):
        if ros2Node is None:
            raise ValueError("No ROS2 node is available for the selected robot")

        topic = self._normalizeTopicName(triggerTopic)
        existingPublisher = ros2Node.GetPublisherNodeByTopic(topic)
        if existingPublisher is not None:
            if self._publisherLooksLikeBool(existingPublisher):
                return existingPublisher, topic, "reused"
            raise RuntimeError(
                f"A publisher already exists on topic '{topic}', but it does not look like "
                "a SlicerROS2 Bool publisher. Remove this publisher in the ROS2 module "
                "or choose a new topic, then try again. "
                f"Publisher class: {self._nodeClassName(existingPublisher)}. "
                f"Blank message: {self._blankMessageDescription(existingPublisher)}. "
                f"Registered publisher nodes: {self._registeredPublisherNodes(ros2Node)}"
            )

        publisherTypeCandidates, registered = self._boolPublisherCandidates(ros2Node)
        errors = []
        for publisherType in publisherTypeCandidates:
            publisher = None
            try:
                publisher = ros2Node.CreateAndAddPublisherNode(publisherType, topic)
            except Exception as exc:
                errors.append(f"{publisherType}: create failed: {exc}")
                continue

            if publisher is None:
                errors.append(f"{publisherType}: CreateAndAddPublisherNode returned None")
                continue

            if self._publisherLooksLikeBool(publisher):
                return publisher, topic, "created"

            errors.append(
                f"{publisherType}: created {self._nodeClassName(publisher)}, "
                f"but blank message is {self._blankMessageDescription(publisher)}"
            )
            self._removePublisherByTopicQuietly(ros2Node, topic)

        raise RuntimeError(
            f"Could not create a compatible Bool publisher for topic '{topic}'. "
            f"Tried: {', '.join(publisherTypeCandidates)}. "
            f"Registered publisher nodes: {registered}. Errors: {'; '.join(errors)}"
        )

    def publishBoolTrigger(self, robotNode, triggerTopic: str, triggerValue: bool) -> dict:
        """Publish a std_msgs/msg/Bool trigger command."""
        if robotNode is None:
            raise ValueError("No robot node selected")

        ros2Node = self._getROS2NodeForRobot(robotNode)
        publisher, normalizedTopic, publisherAction = self._getOrCreateBoolPublisher(ros2Node, triggerTopic)

        value = bool(triggerValue)
        try:
            publisher.Publish(value)
        except TypeError:
            # Fallback for generated message wrappers, if present.
            if not hasattr(publisher, "GetBlankMessage"):
                raise
            message = publisher.GetBlankMessage()
            if hasattr(message, "SetData"):
                message.SetData(value)
            elif hasattr(message, "SetValue"):
                message.SetValue(value)
            else:
                raise
            publisher.Publish(message)

        logging.info(
            "Bool trigger published: robot='%s', topic='%s', value=%s, publisher=%s",
            robotNode.GetName(),
            normalizedTopic,
            value,
            publisherAction,
        )

        return {
            "publisher": publisher,
            "topic": normalizedTopic,
            "publisherAction": publisherAction,
            "value": value,
        }

    def publishDesiredPose(self, robotNode, desiredTransformNode, commandTopic: str, frameId: str) -> dict:
        """Publish the selected transform as a geometry_msgs/msg/PoseStamped command.

        The publisher is created only once per topic on the robot's ROS2 node.
        Subsequent button clicks reuse the existing publisher for the same topic.
        """
        if robotNode is None:
            raise ValueError("No robot node selected")
        if desiredTransformNode is None:
            raise ValueError("No desired-pose transform selected")
        frameId = (frameId or "").strip()

        ros2Node = self._getROS2NodeForRobot(robotNode)
        publisher, normalizedTopic, publisherAction = self._getOrCreatePoseStampedPublisher(ros2Node, commandTopic)

        desiredMatrix = vtk.vtkMatrix4x4()
        desiredTransformNode.GetMatrixTransformToWorld(desiredMatrix)

        # SlicerROS2 PoseStamped publishing pattern, matching the user's working
        # script:
        #   msg = publisher.GetBlankMessage()
        #   msg.GetHeader().SetFrameId(frameId)
        #   msg.SetPose(vtkMatrix4x4)
        #   publisher.Publish(msg)
        if not hasattr(publisher, "GetBlankMessage"):
            raise RuntimeError(
                f"Existing publisher on topic '{normalizedTopic}' does not expose GetBlankMessage(). "
                "It may not be a PoseStamped publisher. Remove that publisher or choose another topic."
            )

        poseMessage = publisher.GetBlankMessage()
        frameIdSet = False

        if isinstance(poseMessage, vtk.vtkMatrix4x4):
            # Permissive fallback for older/alternate builds that expose a raw matrix.
            poseMessage.DeepCopy(desiredMatrix)
            if frameId:
                for setterName in ("SetFrameId", "SetFrameID", "SetFrame_id"):
                    setter = getattr(publisher, setterName, None)
                    if setter is not None:
                        try:
                            setter(frameId)
                            frameIdSet = True
                            break
                        except Exception:
                            pass
        else:
            if not hasattr(poseMessage, "SetPose"):
                raise RuntimeError(
                    f"PoseStamped publisher on topic '{normalizedTopic}' returned an unsupported blank message. "
                    "Expected an object with SetPose(vtkMatrix4x4). "
                    f"Publisher class: {self._nodeClassName(publisher)}. "
                    f"Blank message: {self._blankMessageDescription(publisher)}."
                )

            header = poseMessage.GetHeader() if hasattr(poseMessage, "GetHeader") else None
            if frameId:
                if header is not None and hasattr(header, "SetFrameId"):
                    header.SetFrameId(frameId)
                    frameIdSet = True
                elif header is not None and hasattr(header, "SetFrameID"):
                    header.SetFrameID(frameId)
                    frameIdSet = True

            poseMessage.SetPose(desiredMatrix)

        try:
            publisher.Publish(poseMessage)
        except TypeError as exc:
            raise TypeError(
                "SlicerROS2 PoseStamped publishing failed. The expected pattern is: "
                "message = publisher.GetBlankMessage(); message.GetHeader().SetFrameId(frameId); "
                "message.SetPose(matrix); publisher.Publish(message). The publisher currently "
                "attached to this topic may not be a PoseStamped publisher."
            ) from exc

        logging.info(
            "PoseStamped command published: robot='%s', desiredTransform='%s', topic='%s', "
            "frame_id='%s', publisher=%s, frame_id_set=%s, translation_world_mm=(%.3f, %.3f, %.3f)",
            robotNode.GetName(),
            desiredTransformNode.GetName(),
            normalizedTopic,
            frameId,
            publisherAction,
            frameIdSet,
            desiredMatrix.GetElement(0, 3),
            desiredMatrix.GetElement(1, 3),
            desiredMatrix.GetElement(2, 3),
        )

        return {
            "publisher": publisher,
            "topic": normalizedTopic,
            "publisherAction": publisherAction,
            "frameId": frameId,
            "frameIdSet": frameIdSet,
        }


#
# CustomControlTest
#


class CustomControlTest(ScriptedLoadableModuleTest):
    """Basic smoke test for the Custom Control module."""

    def setUp(self):
        slicer.mrmlScene.Clear()

    def runTest(self):
        self.setUp()
        self.test_CustomControlSmokeTest()

    def test_CustomControlSmokeTest(self):
        self.delayDisplay("Starting Custom Control smoke test")
        logic = CustomControlLogic()
        self.assertIsNotNone(logic.getParameterNode())
        self.delayDisplay("Test passed")
