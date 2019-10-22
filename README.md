# Rowma client js
This package is a nodejs client for rowma.

# Installation
```
$ npm install rowma_js
```

# Example
```nodejs
import Rowma, { getGeohash } from 'rowma_js';

const geohash = await getGeohash();
const rowma = new Rowma(geohash);
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

# Methods API
## constructor(geohash, options = {})
```nodejs
const rowma = new Rowma('geohash')
```

#### Parameters
|Name|Type|Required|Description|
|:-|:-|:-|:-|
|geohash|string|true|A geohash to specify the user|
|options|Object|false||

#### Returns
A Rowma object

## currentConnectionList()
Get a list of connection list.

#### Parameters
No parameters

#### Returns
An axios response object is returned.

## runLaunch(uuid, command)
Send a WebSocket request to the connection manager with uuid and command.

#### Parameters
|Name|Type|Required|Description|
|:-|:-|:-|:-|
|uuid|string|true||
|command|string|true||

#### Returns
No return values

## getGeohash()
Get the geohash of current location from its IP address by freegeoip.app.

#### Parameters
|Name|Type|Required|Description|
|:-|:-|:-|:-|
|precision|number|false|The number of geohash characters|

#### Returns
A geohash string is returned.

# License
MIT © asmsuechan
