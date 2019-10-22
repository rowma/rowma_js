import axios from 'axios';
import io from 'socket.io-client';
import uuidv4 from 'uuid/v4';

class Rowma {
  constructor(geocode, opts = {}) {
    this.geocode = geocode;

    this.baseURL = opts.baseURL || 'http://18.176.1.219';
    this.client = axios.create({
      baseURL: this.baseURL,
      timeout: 1000
    });
    this.uuid = uuidv4();
  }

  currentConnectionList() {
    const path = '/list_connections';
    const params = { geocode: this.geocode };
    return this.client.get(path, { params });
  }

  getRobotStatus(uuid) {
    const path = '/robots';
    const params = { uuid };
    return this.client.get(path, { params });
  }

  runLaunch(socket, uuid, command) {
    return new Promise((resolve, reject) => {
      socket.emit('run_launch', { uuid, command }, (res) => resolve(res));
    });
  }

  runRosrun(socket, uuid, command, args) {
    return new Promise((resolve, reject) => {
      socket.emit('run_rosrun', { uuid, command, args}, (res) => resolve(res));
    });
  }

  killNodes(socket, uuid, rosnodes) {
    return new Promise((resolve, reject) => {
      socket.emit('kill_rosnodes', { uuid, rosnodes }, (res) => resolve(res));
    })
  }

  registerDevice(socket, robotUuid) {
    return new Promise((resolve, reject) => {
      socket.emit('register_device', { deviceUuid: this.uuid, robotUuid }, (res) => resolve(res));
    });
  }

  connect(robotUuid) {
    return new Promise((resolve, reject) => {
      try {
        const socket = io.connect(`${this.baseURL}/conn_device`);
        this.registerDevice(socket, robotUuid).then(res => {
          console.log(res)
        }).catch(e => {
          console.log('error', e)
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

  publishTopic(socket, robotUuid, msg) {
    return new Promise((resolve, reject) => {
      socket.emit('delegate', { robotUuid, msg }, (res) => resolve(res));
    })
  }

  subscribeTopic(socket, robotUuid, topic) {
    // TODO: Make msg JSON string
    const msg =  {
      "op": "subscribe",
      "deviceUuid": this.uuid,
      "topic": topic
    }

    return new Promise((resolve, reject) => {
      socket.emit('delegate', { robotUuid, msg }, (res) => resolve(res));
    })
  }
}

export default Rowma;
