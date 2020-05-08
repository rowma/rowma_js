import axios, { AxiosInstance } from "axios";
import * as io from "socket.io-client";
import * as uuidv4 from "uuid/v4";

interface RowmaOptionsInterface {
  baseURL?: string;
}

// This interface is defined at rowma/rowma/src/entity/response.ts
interface ResponseInterface {
  status: string;
  data: string;
  error: string;
}

class Rowma {
  baseURL: string;
  socket: SocketIOClient.Socket;
  uuid: string;

  private client: AxiosInstance;

  constructor(opts: RowmaOptionsInterface = {}) {
    this.baseURL = opts.baseURL || "https://rowma.moriokalab.com";
    this.client = axios.create({ baseURL: this.baseURL });
    this.uuid = uuidv4();
  }

  /**
   * Get information of Rowma Network.
   * @return {Promise} Return an axios object
   */
  getNetworkInformation() {
    const path = "/network_information";
    return this.client.get(path);
  }

  /**
   * Get the connection list of Rowma Network.
   * @param {string} jwt
   * @param {string} networkUuid
   * @return {Promise} Return an axios object
   */
  currentConnectionList(networkUuid = "default", jwt = "") {
    const opts = {
      headers: {
        Authorization: jwt,
      },
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
  getRobotStatus(uuid: string, networkUuid = "default", jwt = "") {
    const path = "/robots";
    const opts = {
      params: {
        uuid,
        networkUuid,
      },
      headers: {
        Authorization: jwt,
      },
    };
    return this.client.get(path, opts);
  }

  /**
   * Run the specified roslaunch command (e.g. my_utility rviz.launch) on the robot.
   * @param {string} uuid
   * @param {string} command
   * @return {Promise} Return a Promise with a response.
   */
  runLaunch(uuid: string, command: string) {
    return new Promise((resolve) => {
      const destination = { type: "robot", uuid };
      this.socket.emit(
        "run_launch",
        { destination, command },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }

  /**
   * Run the specified rosrun command (e.g. rviz rviz) on the robot.
   * @param {string} uuid
   * @param {string} command
   * @param {string} args Command arguments for the rosrun command.
   * @return {Promise} Return a Promise with a response.
   */
  runRosrun(
    uuid: string,
    command: string,
    args: string
  ) {
    return new Promise((resolve) => {
      const destination = { type: "robot", uuid };
      this.socket.emit(
        "run_rosrun",
        { destination, command, args },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }

  /**
   * Kill the specified ros node running on the robot.
   * @param {string} uuid
   * @param {Array<string>} rosnodes
   * @return {Promise} Return a Promise with a response.
   */
  killNodes(
    uuid: string,
    rosnodes: Array<string>
  ) {
    return new Promise((resolve) => {
      const destination = { type: "robot", uuid };
      this.socket.emit(
        "kill_rosnodes",
        { destination, rosnodes },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }

  /**
   * Match the UUID of a device client and a UUID of the robot on ConnectionManager.
   * @param {string} RobotUUID
   * @return {Promise} Return a Promise with a response.
   */
  registerDevice(robotUuid: string) {
    return new Promise((resolve) => {
      this.socket.emit(
        "register_device",
        { deviceUuid: this.uuid, robotUuid },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }

  /**
   * Connect to ConnectionManager.
   * @param {string} RobotUUID
   * @return {Promise} Return a Promise with a socket for connection.
   */
  connect(robotUuid: string) {
    return new Promise((resolve, reject) => {
      try {
        const socket = io.connect(`${this.baseURL}/rowma`);
        this.registerDevice(socket, robotUuid).catch((e) => {
          // eslint-disable-next-line no-console
          console.error("error", e);
        });

        this.socket = socket;
        resolve();
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
  connectWithAuth(networkUuid: string, robotUuid: string, jwt = "") {
    return new Promise((resolve, reject) => {
      const extraHeaders = { Authorization: jwt, networkUuid };
      try {
        const socket = io.connect(`${this.baseURL}/rowma_device`, {
          // eslint-disable-next-line @typescript-eslint/ban-ts-ignore
          // @ts-ignore
          extraHeaders, // For nodejs
          transportOptions: {
            // For browsers
            polling: {
              extraHeaders,
            },
          },
        });
        socket.on("unauthorized", (error: string) => {
          throw error;
        });
        this.registerDevice(socket, robotUuid).catch((e) => {
          // eslint-disable-next-line no-console
          console.error("error", e);
        });

        this.socket = socket;
        resolve();
      } catch (e) {
        reject(e);
      }
    });
  }

  close() {
    this.socket.close();
  }

  /**
   * Publish a topic which runs on the specified robot.
   * @param {string} RobotUUID
   * @param {string} msg Message for topic
   * @return {Promise} Return a Promise with a response.
   */
  publishTopic(uuid: string, msg: string) {
    return new Promise((resolve) => {
      const destination = { type: "robot", uuid };
      this.socket.emit("delegate", { destination, msg }, (res: ResponseInterface) =>
        resolve(res)
      );
    });
  }

  /**
   * Subscribe a topic.
   * @param {string} destuuid
   * @param {string} destType
   * @param {string} topicDestUuid
   * @param {string} topic
   * @return {Promise} Return a Promise with a response.
   */
  subscribeTopic(
    destUuid: string,
    topicDestType: string,
    topicDestUuid: string,
    topic: string
  ) {
    const destination = { type: "robot", uuid: destUuid };
    const topicDestination = { type: topicDestType, uuid: topicDestUuid };
    const msg = {
      op: "subscribe",
      topicDestination,
      topic,
    };

    return new Promise((resolve) => {
      this.socket.emit("delegate", { destination, msg }, (res: ResponseInterface) =>
        resolve(res)
      );
    });
  }

  /**
   * Subscribe a topic.
   * @param {string} destuuid
   * @param {string} topic
   * @return {Promise} Return a Promise with a response.
   */
  unsubscribeTopic(
    destUuid: string,
    topic: string
  ) {
    const destination = { type: "robot", uuid: destUuid };

    return new Promise((resolve) => {
      this.socket.emit(
        "unsubscribe_rostopic",
        { destination, topic },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }

  /**
   * Get the robot status by UUID from ConnectionManager.
   * @param {string} jwt
   * @param {string} uuid
   * @return {Promise} Return an axios object
   */
  deleteRobot(uuid: string, networkUuid = "default", jwt = "") {
    const path = `/robots/${uuid}`;
    const opts = {
      params: {
        networkUuid,
      },
      headers: {
        Authorization: jwt,
      },
    };
    return this.client.delete(path, opts);
  }

  /**
   * Add a script to the specified robot.
   * @param {string} uuid
   * @param {string} name
   * @param {string} script
   * @return {Promise} Return a Promise with a response.
   */
  addScript(
    uuid: string,
    name: string,
    script: string
  ) {
    return new Promise((resolve) => {
      const destination = { type: "robot", uuid };
      this.socket.emit(
        "add_script",
        { destination, name, script },
        (res: ResponseInterface) => resolve(res)
      );
    });
  }
}

export default Rowma;
