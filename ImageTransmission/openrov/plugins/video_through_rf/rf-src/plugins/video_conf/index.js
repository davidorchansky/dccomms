var net = require('net');

//Modificar las dos lineas siguientes para cambiar el servidor de video
//var videoAddr = "192.168.1.108";
var videoAddr = "localhost";
var videoPort = 8083;



var mparser = new RegExp(/"/g);

function video_conf(name, deps) {
    console.log("This is where video_conf code would execute in the node processa.");
    this.transmissionConfig = 
	{
		frameSize : 100,
		maxPacketLength: 150,
		delayBetweenPackets: 0,
		maxFrameAge: 20,
	};

    var vidconf = this;
    deps.io.sockets.on('connection', function (socket) {

	socket.emit('video_conf_fromServer', vidconf.transmissionConfig);

    	connectToServer(socket, JSON.stringify(vidconf.transmissionConfig).replace(mparser,""));
	
        socket.on('video_conf', function(data){

	    vidconf.transmissionConfig = data;  
	    var sdata = JSON.stringify(data);
	    sdata = sdata.replace(mparser,"");
            console.log('recibido mensaje de configuracion de video: '+sdata);
	    socket.broadcast.emit('video_conf_fromServer', data);

	    if(socket.connectedToServer)
	    {
	    	console.log("Communicating with video conf server");
		socket.videoConnection.write(sdata);	
	    }
	    else
	    {
	    	connectToServer(socket, sdata);
	    }
        });
    });    
     
};

function connectToServer(socket, sdata)
{
	console.log("Trying to connect with video conf. server");
	socket.videoConnection = net.connect({host: videoAddr, port: videoPort},
	    function() { //'connect' listener
	  console.log('connected to server!');
          socket.connectedToServer = true;
	  socket.videoConnection.write(sdata);
	
	});
	socket.videoConnection.on('error', function()
	{
		console.log("Error");
          	socket.connectedToServer = false;
	});

	socket.videoConnection.on('close', function()
	{
		console.log("Connection closed");
          	socket.connectedToServer = false;
	});

	socket.videoConnection.on('data', function(data) {
	  	console.log("Data received from server: "+data.toString());
          	socket.connectedToServer = true;
	});

	socket.videoConnection.on('end', function() {
	  	console.log('disconnected from server');
          	socket.connectedToServer = false;
	});
}
;

module.exports = video_conf;
