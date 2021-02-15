//Part of CanOS
//Heavy copypasta from StackOverflow due to lack-of-time :D
//Anyhow, credit where due
//@Alto

$('.element').bind('click', function() {
	var jump = $(this).attr('data-view');

  us = document.getElementsByClassName("element");
  for (i = 0; i < us.length; i++) {
      us[i].classList.remove('current');
  }
  this.classList.add('current');
  them = document.getElementsByClassName("content");
  for (i = 0; i < us.length; i++) {
      them[i].classList.remove('curr');
  }
  document.getElementById(jump).classList.add('curr');
});

function getTelemetry() {
  const url = 'http://localhost:4170/telemetry'
  fetch(url)
  .then(response => response.json())
  .then(json => {
		if (json.ERROR == "TRUE") {
			return
		}
		insane = "Pass";
		alttext = json.ALT;

    console.log(json);

		if (json.ALT == "----") {
			alttext = "GND"
			insane = "Altitude low";
			json.ALT = 0;
		}
		if (json.ALT == "++++") {
			alttext = "HIGH"
			insane = "Altitude high";

			json.ALT = 9999;
		}
		if (json.VELO == "--") {
			insane = "Velocity negative";
			json.VELO = 0;
		}
		if (json.VELO == "++") {
			insane = "Velocity high";

			json.VELO = 36;
		}

		////
		console.log(json.DATA);
		updateChart(json.DATA);

		document.getElementById('bar').style.height = (json.ALT / 5) + "px";
		document.getElementById('barv').innerHTML = alttext;

		setProgress('fallspd', -1 * json.FALL / 36 * 100, -1 * json.FALL); //we are falling
		setProgress('extime', json.SAMPLING / 90 * 100, json.SAMPLING);
		setProgress('rssi', 100 - Math.abs(json.RSSI), json.RSSI);
		setProgress('gspd', json.VELO / 36 * 100, json.VELO);

		setDefinite('vhead', String(json.DIR) + '°');

		if (json.ERROR == "NULErr") {
			setDefinite('vhealth', "OK");
		} else {
			setDefinite('vhealth', "ERROR");
		}


		setDefinite('posnow', json.LAT/10000 + ", " + json.LON/10000);

		setDefinite('antdata', json.RAW);

		setDefinite('mode', json.MODE);

		mapFlyTo(json.LAT / 10000, json.LON / 10000);


  })



	setTimeout(getTelemetry, 200); //set nice and health no memleak
}

getTelemetry();

function setProgress(elid, percent, raw) {
	if (percent > 100) {
		percent = 100;
	}
	if (percent < 0.1) {
		percent = 0.1;
	}
	var circle = document.getElementById(elid);
	console.log(circle);
	var radius = circle.r.baseVal.value;
	var circumference = radius * 2 * Math.PI;

	circle.style.strokeDasharray = `${circumference} ${circumference}`;
	circle.style.strokeDashoffset = `${circumference}`;

  const offset = circumference - percent / 100 * circumference;
  circle.style.strokeDashoffset = offset;

	document.getElementById(elid + "num").innerHTML = raw;
}

function setDefinite(elid, raw) {
	document.getElementById(elid + "num").innerHTML = raw;
}


//°


//chart like da devilll
// Our labels and three data series
var data = {
	labels: ["A", "B", "C"],
	series: [
		[1, 2, 3],
	]
};

new Chartist.Line('.ct-chart', data, {
  fullWidth: true,
  chartPadding: {
    right: 50,
		left: 20
  }
});


function updateChart(data) {
	chart = document.getElementById("mchart").__chartist__;
	chart.update(data);
}



//mapping

mapboxgl.accessToken = 'pk.eyJ1IjoiY2lyY2xpIiwiYSI6ImNrbDJ0OWxhMDE4bzEyb2xibGZzazh0YXkifQ.8g61d-dwQeKHAst7fbU3KQ';
var map = new mapboxgl.Map({
  container: 'map',
  zoom: 1,
  center: [20.9974, 52.2103],
  style: 'mapbox://styles/mapbox/satellite-v9',
  interactive: false
});

function mapFlyTo(lat, lon) {
	map.flyTo({
	  center: [
	    lon, lat
	  ],
		zoom: 18,
	  essential: false //is nonessenial -lag
	});
}
