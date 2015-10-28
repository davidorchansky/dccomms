(function (window, $, undefined) {
    'use strict';

    var Cockpit;

    Cockpit = function(dashboard) {

        var viewModel = new ProcessModel(); 
    	viewModel.requestStatus = function() { dashboard.socket.emit('status-rfcockpit'); };
    	viewModel.start = function() { dashboard.socket.emit('start-rfcockpit'); };
    	viewModel.stop = function() { dashboard.socket.emit('stop-rfcockpit') };
    
        dashboard.viewModel.rfcockpit = viewModel;

        dashboard.socket.on('status-rfcockpit', function(status) {
        	viewModel.status(status);
        });

        // Add required UI elements
        $("#main-row").append('<div id="rfcockpit"></div>');
        $("#rfcockpit").load(
        	'plugin/04_rfcockpit/plugin.html',
            function() { ko.applyBindings(dashboard.viewModel); }
            );

        setInterval(viewModel.requestStatus, 3000);
        viewModel.requestStatus();

        console.log("Loaded Cockpit plugin.");
    };
    window.Dashboard.plugins.push(Cockpit);

}(window, jQuery));
