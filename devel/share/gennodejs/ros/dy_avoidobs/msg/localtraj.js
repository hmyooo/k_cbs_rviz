// Auto-generated. Do not edit!

// (in-package dy_avoidobs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class localtraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_id = null;
      this.traj_id = null;
      this.start_time = null;
      this.pos_pts = null;
    }
    else {
      if (initObj.hasOwnProperty('car_id')) {
        this.car_id = initObj.car_id
      }
      else {
        this.car_id = 0;
      }
      if (initObj.hasOwnProperty('traj_id')) {
        this.traj_id = initObj.traj_id
      }
      else {
        this.traj_id = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('pos_pts')) {
        this.pos_pts = initObj.pos_pts
      }
      else {
        this.pos_pts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type localtraj
    // Serialize message field [car_id]
    bufferOffset = _serializer.int32(obj.car_id, buffer, bufferOffset);
    // Serialize message field [traj_id]
    bufferOffset = _serializer.int64(obj.traj_id, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [pos_pts]
    // Serialize the length for message field [pos_pts]
    bufferOffset = _serializer.uint32(obj.pos_pts.length, buffer, bufferOffset);
    obj.pos_pts.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type localtraj
    let len;
    let data = new localtraj(null);
    // Deserialize message field [car_id]
    data.car_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [traj_id]
    data.traj_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [pos_pts]
    // Deserialize array length for message field [pos_pts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pos_pts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pos_pts[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.pos_pts.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dy_avoidobs/localtraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3dc77ae9fb11c09e1886c8d863ff6e72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 car_id
    
    int64 traj_id
    time start_time
    
    
    geometry_msgs/Point[] pos_pts
    
    
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new localtraj(null);
    if (msg.car_id !== undefined) {
      resolved.car_id = msg.car_id;
    }
    else {
      resolved.car_id = 0
    }

    if (msg.traj_id !== undefined) {
      resolved.traj_id = msg.traj_id;
    }
    else {
      resolved.traj_id = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.pos_pts !== undefined) {
      resolved.pos_pts = new Array(msg.pos_pts.length);
      for (let i = 0; i < resolved.pos_pts.length; ++i) {
        resolved.pos_pts[i] = geometry_msgs.msg.Point.Resolve(msg.pos_pts[i]);
      }
    }
    else {
      resolved.pos_pts = []
    }

    return resolved;
    }
};

module.exports = localtraj;
