# Rowma SDK js
[![Netlify Status](https://api.netlify.com/api/v1/badges/a3c89089-28b2-459f-b933-4d345e240019/deploy-status)](https://app.netlify.com/sites/upbeat-swanson-9ce1b7/deploys)

This package is a nodejs SDK for rowma.

documentation is available [here](https://upbeat-swanson-9ce1b7.netlify.com/)

## rowma repository
Check [the rowma main repository](https://github.com/asmsuechan/rowma) for more information to utilize rowma.

## Installation
```
$ npm install rowma_js
```

## Example

### Simple example
```nodejs
import Rowma from 'rowma_js';

const rowma = new Rowma();

const robotList = await rowma.currentConnectionList()
const robot = connectionList[0] // Chose a connection

const socket = await rowma.connect(robot)

const command = 'my_utility rviz.launch'
rowma.runLaunch(socket, robot, command)
```

#### Get commands
```nodejs
import Rowma from 'rowma_js';

const rowma = new Rowma();

const robotList = await rowma.currentConnectionList()
const robot = connectionList[0] // Choose a connection

const commands = robot.launchCommands
const command = commands[0] // Choose a command

rowma.runLaunch(socket, robot, command)
```

## License
MIT © asmsuechan
