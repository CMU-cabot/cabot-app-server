/*******************************************************************************
 * Copyright (c) 2023  Carnegie Mellon University and Miraikan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

/**
 * A PoseStamped client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 *  * headLength (optional) - the head length of the arrow (default: 0.2)
 *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
 *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
 */
ROS3D.PoseLog = function(options) {
  THREE.Object3D.call(this);
  this.options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/pose';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.throttle_rate = options.throttle_rate || 0;

  this.sceneNode = null;
  this.currentPose = null;

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.PoseLog.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.PoseLog.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.PoseLog.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      throttle_rate: this.throttle_rate,
      queue_length : 1,
      messageType : 'cabot_msgs/msg/PoseLog'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.PoseLog.prototype.processMessage = function(message){
  this.options.origin = new THREE.Vector3( message.pose.position.x, message.pose.position.y,
                                           message.pose.position.z);

  var rot = new THREE.Quaternion(message.pose.orientation.x, message.pose.orientation.y,
                                 message.pose.orientation.z, message.pose.orientation.w);
  this.options.direction = new THREE.Vector3(1,0,0);
  this.options.direction.applyQuaternion(rot);
  this.options.material = new THREE.MeshBasicMaterial({color: this.color});
  var arrow = new ROS3D.Arrow(this.options);

  if (this.sceneNode == null) {
    this.sceneNode = new ROS3D.SceneNode({
      frameID: message.header.frame_id,
      tfClient: this.tfClient,
      object: arrow
    });
    this.rootObject.add(this.sceneNode);
    this.currentPose = arrow;
  } else {
    this.sceneNode.remove(this.currentPose);
    this.currentPose.dispose();
    this.sceneNode.add(arrow);
    this.currentPose = arrow;
  }

  if (this.onMessage) {
     this.onMessage(message.pose);
  }
};


/**
 * @fileOverview
 * @author yoshizawa
 */

/**
 * A People client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * radius (optional) - radius of the point (default: 0.2)
 */
ROS3D.People = function(options) {
  THREE.Object3D.call(this);
  this.options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/people';
  this.tfClient = options.tfClient;
  this.color = options.color || 0x0000ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.radius = options.radius || 0.2;
  this.throttle_rate = options.throttle_rate || 0;

  this.sceneNode = null;
  this.people = [];

  this.rosTopic = undefined;
  this.subscribe();
};


ROS3D.People.prototype.__proto__ = THREE.Object3D.prototype;

ROS3D.People.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.People.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      throttle_rate: this.throttle_rate,
      queue_length : 1,
      messageType : 'people_msgs/msg/People'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.People.prototype.processMessage = function(message){

  for (let i = 0; i < this.people.length; i++) {
      this.sceneNode.remove(this.people[i]);
      this.people[i].geometry.dispose();
      this.people[i].material.dispose();
  }

  if (message.people.length > 0) {
      this.people = [];

      message.people.forEach(person => {
          var sphereGeometry = new THREE.SphereGeometry( this.radius );
          var sphereMaterial = new THREE.MeshBasicMaterial( {color: this.color} );
          var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
          sphere.position.set(person.position.x, person.position.y, person.position.z);

          if (this.sceneNode == null) {
            this.sceneNode = new ROS3D.SceneNode({
              frameID: message.header.frame_id,
              tfClient: this.tfClient,
              object: sphere
            });
            this.rootObject.add(this.sceneNode);
            this.people.push(sphere);
          } else {
            this.sceneNode.add(sphere);
            this.people.push(sphere);
          }
      });

  }
};
