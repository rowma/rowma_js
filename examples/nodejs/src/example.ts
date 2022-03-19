import { Rowma } from 'rowma_js';

const rowma = new Rowma();

const conns = await rowma.currentConnectionList()
const robotList = conns.data
const robot = robotList[0]

const socket = await rowma.connect()

const command = 'my_utility rviz.launch'
// rowma.runLaunch(socket, robot, command)
