# Rowma client js
This package is a nodejs client for rowma.

documentation is available [here](https://upbeat-swanson-9ce1b7.netlify.com/)

# Installation
```
$ npm install rowma_js
```

# Example
```nodejs
import Rowma from 'rowma_js';

const rowma = new Rowma();
let connectionList = []
rowma.currentConnectionList().then(res => {
  connectionList = res.data
})
const connection = connectionList[0] // Choose a connection
const uuid = connection.uuid
const commands = connection.launchCommands
const command = commands[0] // Choose a command
rowma.runLaunch(connectionUuid, command)
```

# License
MIT © asmsuechan
