import axios from 'axios';
import io from 'socket.io-client';
import uuidv4 from 'uuid/v4';

class Rowma {
  constructor(opts = {}) {
    this.baseURL = opts.baseURL || 'http://18.176.1.219';
    this.client = axios.create({
      baseURL: this.baseURL,
      timeout: 1000
    });
    this.uuid = uuidv4();
  }

  currentConnectionList() {
    const path = '/list_connections';
    return this.client.get(path);
  }

  /**
   * Get the robot status by UUID from ConnectionManager.
   * @param {string} uuid
   * @return {Promise} Return an axios object
   */
  getRobotStatus(uuid) {
    const path = '/robots';
    const params = { uuid };
    return this.client.get(path, { params });
  }

  /**
   * Run the specified roslaunch command (e.g. my_utility rviz.launch) on the robot.
   * @param {socket} socket
   * @param {string} uuid
   * @param {string} command
   * @return {Promise} Return a Promise with a response.
   */
  runLaunch(socket, uuid, command) {
    return new Promise((resolve) => {
      socket.emit('run_launch', { uuid, command }, res => resolve(res));
    });
  }

  /**
   * Run the specified rosrun command (e.g. rviz rviz) on the robot.
   * @param {socket} socket
   * @param {string} uuid
   * @param {string} command
   * @param {string} args Command arguments for the rosrun command.
   * @return {Promise} Return a Promise with a response.
   */
  runRosrun(socket, uuid, command, args) {
    return new Promise((resolve) => {
      socket.emit('run_rosrun', { uuid, command, args }, res => resolve(res));
    });
  }

  /**
   * Kill the specified ros node running on the robot.
   * @param {socket} socket
   * @param {string} uuid
   * @param {Array<string>} rosnodes
   * @return {Promise} Return a Promise with a response.
   */
  killNodes(socket, uuid, rosnodes) {
    return new Promise((resolve) => {
      socket.emit('kill_rosnodes', { uuid, rosnodes }, res => resolve(res));
    });
  }

  /**
   * Match the UUID of a device client and a UUID of the robot on ConnectionManager.
   * @param {socket} socket
   * @param {string} RobotUUID
   * @return {Promise} Return a Promise with a response.
   */
  registerDevice(socket, robotUuid) {
    return new Promise((resolve) => {
      socket.emit('register_device', { deviceUuid: this.uuid, robotUuid }, res => resolve(res));
    });
  }

  /**
   * Connect to ConnectionManager.
   * @param {string} RobotUUID
   * @return {Promise} Return a Promise with a socket for connection.
   */
  connect(robotUuid) {
    return new Promise((resolve, reject) => {
      try {
        const socket = io.connect(`${this.baseURL}/rowma`);
        this.registerDevice(socket, robotUuid).then((res) => {
          console.log(res);
        }).catch((e) => {
          console.log('error', e);
        });

        resolve(socket);
      } catch (e) {
        reject(e);
      }
    });
  }

  close(socket) {
    socket.close();
  }

  /**
   * Publish a topic which runs on the specified robot.
   * @param {socket} socket
   * @param {string} RobotUUID
   * @param {string} msg Message for topic
   * @return {Promise} Return a Promise with a response.
   */
  publishTopic(socket, robotUuid, msg) {
    return new Promise((resolve) => {
      socket.emit('delegate', { robotUuid, msg }, res => resolve(res));
    });
  }

  /**
   * Subscribe a topic.
   * @param {socket} socket
   * @param {string} RobotUUID
   * @param {string} topic
   * @return {Promise} Return a Promise with a response.
   */
  subscribeTopic(socket, robotUuid, topic) {
    // TODO: Make msg JSON string
    const msg = {
      op: 'subscribe',
      deviceUuid: this.uuid,
      topic
    };

    return new Promise((resolve) => {
      socket.emit('delegate', { robotUuid, msg }, res => resolve(res));
    });
  }
}

export default Rowma;
