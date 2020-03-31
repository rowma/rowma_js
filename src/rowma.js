import axios from 'axios';
import io from 'socket.io-client';
import uuidv4 from 'uuid/v4';

class Rowma {
  constructor(opts = {}) {
    this.baseURL = opts.baseURL || 'https://rocky-peak-54058.herokuapp.com';
    this.client = axios.create({ baseURL: this.baseURL });
    this.uuid = uuidv4();
  }

  /**
   * Get information of Rowma Network.
   * @return {Promise} Return an axios object
   */
  getNetworkInformation() {
    const path = '/network_information';
    return this.client.get(path);
  }

  /**
   * Get the connection list of Rowma Network.
   * @param {string} jwt
   * @param {string} networkUuid
   * @return {Promise} Return an axios object
   */
  currentConnectionList(jwt, networkUuid = 'default') {
    const opts = {
      headers: {
        Authorization: jwt
      }
    };
    const path = `/list_connections?uuid=${networkUuid}`;
    return this.client.get(path, opts);
  }

  /**
   * Get the robot status by UUID from ConnectionManager.
   * @param {string} jwt
   * @param {string} uuid
   * @return {Promise} Return an axios object
   */
  getRobotStatus(jwt, uuid, networkUuid = 'default') {
    const path = '/robots';
    const opts = {
      params: {
        uuid,
        networkUuid
      },
      headers: {
        Authorization: jwt
      }
    };
    return this.client.get(path, opts);
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
      const destination = { type: 'robot', uuid };
      socket.emit('run_launch', { destination, command }, res => resolve(res));
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
      const destination = { type: 'robot', uuid };
      socket.emit('run_rosrun', { destination, command, args }, res => resolve(res));
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
      const destination = { type: 'robot', uuid };
      socket.emit('kill_rosnodes', { destination, rosnodes }, res => resolve(res));
    });
  }

  /**
   * Match the UUID of a device client and a UUID of the robot on ConnectionManager.
   * @param {socket} socket
   * @param {string} RobotUUID
   * @return {Promise} Return a Promise with a response.
   */
  registerDevice(socket) {
    return new Promise((resolve) => {
      socket.emit('register_device', { deviceUuid: this.uuid }, res => resolve(res));
    });
  }

  /**
   * Connect to ConnectionManager.
   * @return {Promise} Return a Promise with a socket for connection.
   */
  connect() {
    return new Promise((resolve, reject) => {
      try {
        const socket = io.connect(`${this.baseURL}/rowma`);
        this.registerDevice(socket).then((res) => {
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

  /**
   * Connect to ConnectionManager with Auth information.
   * @param {jwt} JWT token
   * @param {networkId} Network ID
   * @return {Promise} Return a Promise with a socket for connection.
   */
  connectWithAuth(jwt, networkUuid) {
    return new Promise((resolve, reject) => {
      const extraHeaders = { Authorization: jwt, networkUuid };
      try {
        const socket = io.connect(`${this.baseURL}/rowma_device`, {
          extraHeaders, // For nodejs
          transportOptions: { // For browsers
            polling: {
              extraHeaders
            }
          }
        });
        socket.on('unauthorized', (error) => {
          throw error;
        });
        this.registerDevice(socket).then((res) => {
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
  publishTopic(socket, uuid, msg) {
    return new Promise((resolve) => {
      const destination = { type: 'robot', uuid };
      socket.emit('delegate', { destination, msg }, res => resolve(res));
    });
  }

  /**
   * Subscribe a topic.
   * @param {socket} socket
   * @param {string} destuuid
   * @param {string} destType
   * @param {string} topicDestUuid
   * @param {string} topic
   * @return {Promise} Return a Promise with a response.
   */
  subscribeTopic(socket, destUuid, topicDestType, topicDestUuid, topic) {
    const destination = { type: 'robot', uuid: destUuid };
    const topicDestination = { type: topicDestType, uuid: topicDestUuid };
    const msg = {
      op: 'subscribe',
      topicDestination,
      topic
    };

    return new Promise((resolve) => {
      socket.emit('delegate', { destination, msg }, res => resolve(res));
    });
  }

  /**
   * Subscribe a topic.
   * @param {socket} socket
   * @param {string} destuuid
   * @param {string} topic
   * @return {Promise} Return a Promise with a response.
   */
  unsubscribeTopic(socket, destUuid, topic) {
    const destination = { type: 'robot', uuid: destUuid };

    return new Promise((resolve) => {
      socket.emit('unsubscribe_rostopic', { destination, topic }, res => resolve(res));
    });
  }
}

export default Rowma;
