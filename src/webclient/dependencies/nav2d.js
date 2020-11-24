/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

var NAV2D = NAV2D || {
  REVISION : '0.3.0'
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map meta data topic to listen to
 *   * image - the URL of the image to render
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * viewer - the main viewer to render to
 */
NAV2D.ImageMapClientNav = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map_metadata';
  var image = options.image;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || false;

  this.navigator = null;

  // setup a client to get the map
  var client = new ROS2D.ImageMapClient({
    ros : this.ros,
    rootObject : this.rootObject,
    topic : topic,
    image : image
  });
  client.on('change', function() {
    that.navigator = new NAV2D.Navigator({
      ros : that.ros,
      serverName : that.serverName,
      actionName : that.actionName,
      rootObject : that.rootObject,
      withOrientation : that.withOrientation
    });

    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentImage.width, client.currentImage.height);
    that.viewer.shift(client.currentImage.pose.position.x, client.currentImage.pose.position.y);
  });
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * serverName (optional) - the action server name to use for navigation (default '/move_base')
 *   * actionName (optional) - the navigation action name (default: 'move_base_msgs/MoveBaseAction')
 *   * poseTopicName (optional) - the topic name to get robot position (default '/robot_pose')
 *   * poseMessageType (optional) - poseTopicName's message type (default 'geometry_msgs/Pose')
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * index (optional) - useful if multiple robots are present
 */
NAV2D.Navigator = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  var poseTopicName = options.poseTopicName || '/robot_pose';
  var poseMessageType = options.poseMessageType || 'geometry_msgs/Pose';
  var withOrientation = options.withOrientation || false;
  var index = options.index;
  this.rootObject = options.rootObject || new createjs.Container();

  // setup the actionlib client
  var actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });

  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param pose - the goal pose
   * @param withoutMarker - if set to true, do not create a marker for the goal on the map
   */
  function sendGoal(pose, withoutMarker) {
    console.log('sending goal to robot ' + (index+1));

    // create a goal and send it to robot ${index}
    var goal = new ROSLIB.Goal({
      actionClient : actionClient,
      goalMessage : {
        target_pose : {
          header : {
            frame_id : '/map'
          },
          pose : pose
        }
      }
    });
    currentGoal = goal;
    goal.send();

    // create a marker for the goal and put it on the canvas
    if(!withoutMarker) {
      var goalMarker = createMarker(pose);
      that.rootObject.addChild(goalMarker);
    } 

    // handle goal result
    goal.on('result', function() {
      that.rootObject.removeChild(goalMarker);
      // if patrol is active, send robot to next goal on the patrol's coords list
      if(that.patrol && that.patrol.active) {
        var coordIndex = that.patrol.nextIndex;
        var pose = createPoseMessage(that.patrol.coords[coordIndex].x, that.patrol.coords[coordIndex].y, that.patrol.coords[coordIndex].theta);
        that.patrol.nextIndex++;
        that.patrol.nextIndex = that.patrol.nextIndex % that.patrol.coords.length;
        sendGoal(pose, true);
      }
    });
  }

   /**
   * Create a marker on the map.
   *
   * @param pose - the pose that the marker will be based upon
   * @param customConfig - optional config for the marker
   */
  function createMarker(pose, customConfig) {
    var defaultConfig = {
      size : 15,
      strokeSize : 1,
      fillColor : createjs.Graphics.getRGB(255, 64, 128, 0.66),
      pulse : true
    }
    var config = customConfig || defaultConfig;
    var marker = new ROS2D.NavigationArrow(config);
    marker.x = pose.position.x;
    marker.y = -pose.position.y;
    marker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    marker.scaleX = 0.5 / stage.scaleX;
    marker.scaleY = 0.5 / stage.scaleY;
    return marker;
  }

  /**
   * Get z and w orientation params of the quaternion
   *
   * @param theta - orientation of the robot in radians (optional)
   */
  function getOrientationParams(theta) {
    if (theta >= 0 && theta <= Math.PI) {
      theta += (3 * Math.PI / 2);
    } else {
      theta -= (Math.PI/2);
    }
    var qz =  Math.sin(-theta/2.0);
    var qw =  Math.cos(-theta/2.0);
    return {qz: qz, qw: qw};
  }

  /**
   * Create a pose message to send to the robot
   *
   * @param x - position x of the map
   * @param y - position y of the map
   * @param theta - orientation of the robot in radians (optional)
   */
  
  function createPoseMessage(x, y, theta) {
    console.warn(theta);
    // convert map coordinates to ROS coordinates
    var coords = stage.globalToRos(x, y);
    var config = {
      position : new ROSLIB.Vector3(coords)
    }
    if(theta) {
      var orientation = getOrientationParams(theta);
      config.orientation = new ROSLIB.Quaternion({x:0, y:0, z: orientation.qz, w: orientation.qw});
    }
    return new ROSLIB.Pose(config);
  }

  /**
   * Begins patrol on the robot 
   *
   * @param posePositions - coordinate list where the robot will check
   */
  function startPatrol(posePositions) {
    if(posePositions.length === 0) return;

    // set patrol configuration
    that.patrol = {
      active: true,
      coords: posePositions,
      nextIndex: 0
    };

    // send robot to first coordinate
    var pose = createPoseMessage(posePositions[0].x, posePositions[0].y, posePositions[0].theta);
    sendGoal(pose, true);
  }

  that.startPatrol = startPatrol; // expose patrol method to the application

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = new ROS2D.NavigationArrow({
    size : 25,
    strokeSize : 1,
    fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
    pulse : true
  });
  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : poseTopicName,
    messageType : poseMessageType,
    throttle_rate : 100
  });
  poseListener.subscribe(function(pose) {
    // workaround to handle different types of poseMessageType
    var poseData = null;
    if(pose.position) {
      poseData = pose;
    } else if(pose.pose && pose.pose.position) {
      poseData = pose.pose;
    } else if(pose.pose && pose.pose.pose && pose.pose.pose.position) {
      poseData = pose.pose.pose;
    }
    // update the robots position on the map
    robotMarker.x = poseData.position.x;
    robotMarker.y = -poseData.position.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 0.5 / stage.scaleX;
      robotMarker.scaleY = 0.5 / stage.scaleY;
      initScaleSet = true;
    }

    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(poseData.orientation);

    robotMarker.visible = true;
  });

  if (withOrientation === false){
    // setup a double click listener (no orientation)
    this.rootObject.addEventListener('dblclick', function(event) {
      var pose = createPoseMessage(event.stageX, event.stageY);
      // send the goal if this robot is the one selected
      // without this check, every robot would receive the goal
      if(index === window.selectedRobotIndex) {
        sendGoal(pose);
      }
    });
  } else { // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var positionVec3 = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function(event, mouseState) {

      if (mouseState === 'down'){
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
        mouseDown = true;
      }
      else if (mouseState === 'move'){
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);
        
        if ( mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          orientationMarker = new ROS2D.NavigationArrow({
            size : 25,
            strokeSize : 1,
            fillColor : createjs.Graphics.getRGB(0, 255, 0, 0.66),
            pulse : false
          });

          xDelta =  currentPosVec3.x - positionVec3.x;
          yDelta =  currentPosVec3.y - positionVec3.y;
          
          thetaRadians  = Math.atan2(xDelta,yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);
          
          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          orientationMarker.x =  positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 0.5 / stage.scaleX;
          orientationMarker.scaleY = 0.5 / stage.scaleY;
          
          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseDown) { // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        // calculate orientation
        var goalPos = stage.globalToRos(event.stageX, event.stageY);
        var goalPosVec3 = new ROSLIB.Vector3(goalPos);
        xDelta =  goalPosVec3.x - positionVec3.x;
        yDelta =  goalPosVec3.y - positionVec3.y;
        thetaRadians  = Math.atan2(xDelta,yDelta);
        var orientationParams = getOrientationParams(thetaRadians);
        var orientation = new ROSLIB.Quaternion({x:0, y:0, z: orientationParams.qz, w: orientationParams.qw});
        
        // send goal
        var pose = new ROSLIB.Pose({
          position :    positionVec3,
          orientation : orientation
        });
        // send the goal if this robot is the one selected
        // without this check, every robot would receive the goal
        if(index === window.selectedRobotIndex) {
          sendGoal(pose);
        }
      }
    };

    this.rootObject.addEventListener('stagemousedown', function(event) {
      mouseEventHandler(event,'down');
    });

    this.rootObject.addEventListener('stagemousemove', function(event) {
      mouseEventHandler(event,'move');
    });

    this.rootObject.addEventListener('stagemouseup', function(event) {
      mouseEventHandler(event,'up');
    });
  }
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * rootObject (optional) - the root object to add this marker to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * serverName (optional) - the action server name to use for navigation (default: '/move_base')
 *                             can be a string (single robot) or an array of strings (multiple robots)
 *   * poseTopicName (optional) - the topic name to get robot position (default '/robot_pose')
 *                                can be a string (single robot) or an array of strings (multiple robots)  
 *   * poseMessageType (optional) - poseTopicName's message type (default 'geometry_msgs/Pose')
 *   * actionName (optional) - the navigation action name (default: 'move_base_msgs/MoveBaseAction')
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * viewer - the main viewer to render to
 */
NAV2D.OccupancyGridClientNav = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map';
  var continuous = options.continuous;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.poseTopicName = options.poseTopicName || '/robot_pose';
  this.poseMessageType = options.poseMessageType || 'geometry_msgs/Pose';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || false;

  this.navigator1 = null;
  this.navigator2 = null;

  // setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros : this.ros,
    rootObject : this.rootObject,
    continuous : continuous,
    topic : topic
  });
  client.on('change', function() {
    // create one navigator for each robot
    if(typeof that.serverName === 'object' && that.serverName != null) {
      var navigatorKey = 'navigator';
      for(var i=0; i<that.serverName.length; i++) {
        that[navigatorKey+(i+1)] = new NAV2D.Navigator({
          ros : that.ros,
          serverName : that.serverName[i],
          actionName : that.actionName,
          rootObject : that.rootObject,
          withOrientation : that.withOrientation,
          poseTopicName: that.poseTopicName[i],
          poseMessageType: that.poseMessageType,
          index: i
        });
      }
    }
    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentGrid.width, client.currentGrid.height);
    that.viewer.shift(client.currentGrid.pose.position.x, client.currentGrid.pose.position.y);
  });

  return that;
};