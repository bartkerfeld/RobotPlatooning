// Auto-generated. Do not edit!

// (in-package nav_common.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class movement_request {
  constructor() {
    this.category = '';
    this.subclass = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type movement_request
    // Serialize message field [category]
    bufferInfo = _serializer.string(obj.category, bufferInfo);
    // Serialize message field [subclass]
    bufferInfo = _serializer.string(obj.subclass, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type movement_request
    let tmp;
    let len;
    let data = new movement_request();
    // Deserialize message field [category]
    tmp = _deserializer.string(buffer);
    data.category = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [subclass]
    tmp = _deserializer.string(buffer);
    data.subclass = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'nav_common/movement_request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f034dc06870086475e1cf8ef445bd4da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string category
    string subclass
    
    `;
  }

};

module.exports = movement_request;
