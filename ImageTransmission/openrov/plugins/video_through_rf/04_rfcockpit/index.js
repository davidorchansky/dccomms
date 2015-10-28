function cockpit(name, deps) {
    
	deps.io.sockets.on('connection', function (socket) {

		socket.on('status-rfcockpit', function(){
			deps.dashboardEngine.emit('signal', { key: 'status-rfcockpit' });
		});

		socket.on('start-rfcockpit', function() {
			deps.dashboardEngine.emit('signal', { key: 'start-rfcockpit' });
		});

		socket.on('stop-rfcockpit', function() {
			deps.dashboardEngine.emit('signal', { key: 'stop-rfcockpit' });
		});

	});
};

module.exports = cockpit;
