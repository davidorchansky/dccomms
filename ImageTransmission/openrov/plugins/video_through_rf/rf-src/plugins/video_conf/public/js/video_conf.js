/*jshint multistr: true*/
/*jslint es5: true */

(function (window, $, undefined) {
    'use strict';

    var VideoConf;

    VideoConf = function VideoConf(cockpit) {
        console.log("Loading Video_conf plugin in the browser.");

        // Instance variables
        this.cockpit = cockpit;
	this.frameSize = ko.observable(300);
	this.maxPacketLength = ko.observable(400);
	this.maxFrameAge = ko.observable(20);
	this.delayBetweenPackets = ko.observable(0);

	$("#settings H3:contains('Settings')").after('\
		<div id="videoconf"> \
			<h4>Video configuration:</h4> \
\
			<div class="control-group invisible-field"> \
			    <label class="control-label" for="frameSize">Frame size (bytes): </label> \
			    <input type="text" data-bind="value: frameSize" /> \
			    <div class="controls"> \
				<div id="frameSize"></div> \
			    </div> \
			</div> \
			<div class="control-group invisible-field"> \
			    <label class="control-label" for="maxPacketLength">Max. Packet Length (bytes): </label> \
			    <input type="text" data-bind="value: maxPacketLength" /> \
			    <div class="controls"> \
				<div id="maxPacketLength"></div> \
			    </div> \
			</div>\
\
			<div class="control-group invisible-field"> \
			    <label class="control-label" for="delayBetweenPackets">Delay Between Packets (ms): </label> \
			    <input type="text" data-bind="value: delayBetweenPackets" /> \
			    <div class="controls"> \
				<div id="delayBetweenPackets"></div> \
			    </div> \
			</div>\
\
			<div class="control-group invisible-field"> \
			    <label class="control-label" for="maxFrameAge">Max. Frame Age (ms): </label> \
			    <input type="text" data-bind="value: maxFrameAge" /> \
			    <div class="controls"> \
				<div id="maxFrameAge"></div> \
			    </div> \
			</div>\
\
		</div>'
	);

	ko.applyBindings(this,$('#videoconf')[0]);
	var vidconf = this;
	$("#frameSize").slider({
	  min:60,
	  max:12000,
	  value: vidconf.frameSize(),
	  step: 1,
	  slide: function( event, ui ) {
	    vidconf.frameSize( ui.value );
	  }
	});

	$("#maxPacketLength").slider({

	  min:60,
	  max:2000,
	  value: vidconf.maxPacketLength(),
	  step: 1,
	  slide: function( event, ui ) {
	    vidconf.maxPacketLength( ui.value );
	  }
	});

	$("#maxFrameAge").slider({

	  min:0,
	  max:5000,
	  value: vidconf.maxFrameAge(),
	  step: 1,
	  slide: function( event, ui ) {
	    vidconf.maxFrameAge( ui.value );
	  }
	});

	$("#delayBetweenPackets").slider({

	  min:0,
	  max:5000,
	  value: vidconf.delayBetweenPackets(),
	  step: 1,
	  slide: function( event, ui ) {
	    vidconf.delayBetweenPackets( ui.value );
	  }
	});

	VideoConf.prototype.sendVideoConf = function()
	{
		if(!vidconf.confReceivedFromServer)
		{
			console.log("sending video conf.");
			this.cockpit.socket.emit('video_conf',
				{
					frameSize : String(vidconf.frameSize()),
					maxPacketLength : String(vidconf.maxPacketLength()),
					delayBetweenPackets : String(vidconf.delayBetweenPackets()),
					maxFrameAge : String(vidconf.maxFrameAge())
				});
		}
	}

	vidconf.confReceivedFromServer = false;
	//register events
	vidconf.frameSize.subscribe(function(newValue) {
          $("#frameSize").slider("value", newValue);
	  vidconf.sendVideoConf();
	});
	vidconf.maxPacketLength.subscribe(function(newValue) {
          $("#maxPacketLength").slider("value", newValue);
	  vidconf.sendVideoConf();
	});
	vidconf.delayBetweenPackets.subscribe(function(newValue) {
          $("#delayBetweenPackets").slider("value", newValue);
	  vidconf.sendVideoConf();
	});
	vidconf.maxFrameAge.subscribe(function(newValue) {
          $("#maxFrameAge").slider("value", newValue);
	  vidconf.sendVideoConf();
	});


	vidconf.cockpit.socket.on('video_conf_fromServer', function(data)
	 {
		console.log("New video conf received from server!: "+ data);
		vidconf.confReceivedFromServer = true;
		vidconf.frameSize(data.frameSize);
		vidconf.maxPacketLength(data.maxPacketLength);
		vidconf.delayBetweenPackets(data.delayBetweenPackets);
		vidconf.maxFrameAge(data.maxFrameAge);
		vidconf.confReceivedFromServer = false;
	 });





    };

    

    window.Cockpit.plugins.push(VideoConf);

}(window, jQuery));
