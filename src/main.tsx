import 'maplibre-gl/dist/maplibre-gl.css';
import { Protocol } from 'pmtiles';
import maplibregl from 'maplibre-gl';
import * as ROSLIB from 'roslib';
import { Matrix, inverse } from 'ml-matrix';

var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

let graph_visual = new ROSLIB.Topic({
  ros : ros,
  name : '/graph_visual',
  messageType : 'visualization_msgs/msg/MarkerArray'
});
let visual_path = new ROSLIB.Topic({
	ros:ros,
	name: '/visual_path',
	messageType: 'visualization_msgs/msg/MarkerArray'
});

let limited_pose = new ROSLIB.Topic({
	ros:ros,
	name: '/limited_pose',
	messageType: 'geometry_msgs/msg/PoseStamped'
});
let vehicle_state = new ROSLIB.Topic({
	ros:ros,
	name: '/vehicle_state',
	messageType: 'navigation_interface/msg/VehicleState'
});



let protocol = new Protocol();
maplibregl.addProtocol("pmtiles",protocol.tile);
let map = new maplibregl.Map({
	container: 'map',
	style:"osm-liberty/style.json",
	center: [-78.869914,38.435491],
	zoom: 16,
});

let nav = new maplibregl.NavigationControl();
map.addControl(nav, 'top-left');

let A = new Matrix([[ -0.00000156,   0.00001108, -78.86214758],
 [ -0.00000849,  -0.00000137,  38.43388357],
 [  0,           0,           1        ]]);
let B = inverse(A);
	function T(position){
		const {x,y} = position;
		const [x2,y2,z] = A.mmul(Matrix.columnVector([x,y,1])).to1DArray();
		return [x2,y2];
	};
	function Y(lngLat){
		const {lat,lng} = lngLat;
		const [x2,y2,z] = B.mmul(Matrix.columnVector([lng,lat,1])).to1DArray();
		return [x2,y2];
	};
map.on('load', async () => {

let point= (x,y)=> ({
		'type': 'Feature',
                'geometry': {
			'type': 'Point',
			'coordinates': [x,y]
		}
		});

const image = await map.loadImage('https://maplibre.org/maplibre-gl-js/docs/assets/osgeo-logo.png');
        map.addImage('custom-marker', image.data);
map.addSource('limited_pose', {
            'type': 'geojson',
            'data': point(-78.869914,38.435491)	});

function LineString(coordinates){
	return {
	'type': 'Feature',
	'geometry': {
		'type':'LineString',
		'coordinates': coordinates
	}		
};

}
map.addSource('visual_path',{
	'type':'geojson',
	'data': LineString([])
});
map.addSource('remaining_path',{
	'type':'geojson',
	'data': LineString([])
});

map.addLayer({
		'id':'visual_path',
		'type':'line',
		'source':'visual_path',
		'layout': {
                'line-join': 'round',
                'line-cap': 'round'
            },
            'paint': {
                'line-color': '#bdcff0', // '#6495ED',
		'line-opacity': 1,
                "line-width": {
          "base": 1.4,
          "stops": [
            [14, 0],
            [20, 18]
          ]
        }
            }
	});
map.addLayer({
		'id':'remaining_path',
		'type':'line',
		'source':'remaining_path',
		'layout': {
                'line-join': 'round',
                'line-cap': 'round'
            },
            'paint': {
                'line-color': '#6495ED',
		'line-opacity': 1,
                "line-width": {
          "base": 1.4,
          "stops": [
            [14, 0],
            [20, 18]
          ]
        }
            }
	});

let visual_path_coordinates = [];
visual_path.subscribe(function({markers}){
	visual_path_coordinates = markers.map((m)=>T(m.pose.position));
	map.getSource('visual_path').setData(LineString(visual_path_coordinates));
});

let gps_request = new ROSLIB.Topic({
  ros : ros,
  name : '/gps_request',
  messageType : 'navigation_interface/msg/LatLongPoint'
});


function navigateTo(lat,lng){
	let target = new ROSLIB.Message({
		latitude: lat,
		longitude: lng,
		elevation: 0
	});
	gps_request.publish(target);


}
let stop = new ROSLIB.Topic({
  ros : ros,
  name : '/stop',
  messageType : 'navigation_interface/msg/Stop'
});


function setStopped(value){
	let msg = new ROSLIB.Message({
		stop: value,
		distance: 5
	});
	stop.publish(msg);


}
// parseFloat([...document.getElementById("destinations").getElementsByTagName("li")][0].getAttribute("lat"))

const goButton = document.getElementById("go");
const pullOverButton = document.getElementById("pull-over");
let state = {
  "is_navigating": false,
  "reached_destination": true,
  "stopped": false
};

[...document.getElementById("destinations").getElementsByTagName("li")].forEach((el)=>{
	const lat = parseFloat(el.getAttribute("lat"));
	const long = parseFloat(el.getAttribute("long"));
	el.addEventListener("click",()=>{
	
	console.log(lat,long);
	if(!state.is_navigating){
		navigateTo(lat,long);
		[...document.getElementById("destinations").getElementsByTagName("li")].forEach((el2)=>{
			el2.classList.remove("selected");
		});

		el.classList.add("selected");
		}
	});
});


pullOverButton.addEventListener("click", ()=>{
	console.log("PullOver clicked");
	setStopped(true);
	
});
goButton.addEventListener("click", ()=>{
	console.log("Go clicked");
	setStopped(false);
	
});


vehicle_state.subscribe(function(message){
	state = message;
	console.log(message);
	if(message.reached_destination){
		
		map.getSource('remaining_path').setData(LineString([]));

		//map.getSource('visual_path').setData(LineString([]));
	}

	goButton.hidden=!message.stopped || !message.is_navigating;
	pullOverButton.hidden=message.stopped || !message.is_navigating;


});


limited_pose.subscribe(function(message){
	let [x1,y1] =T(message.pose.position);
	let source = map.getSource('limited_pose').setData(point(x1,y1));

	if(visual_path_coordinates.length>0 && state.is_navigating){
		let closestInd = 0;
		let closestDist = Infinity;
		for(let i=0; i < visual_path_coordinates.length ; i++){
			const [x2,y2] = visual_path_coordinates[i];
			const dist = Math.sqrt(Math.pow(x1-x2,2)+Math.pow(y1-y2,2));
			if(dist < closestDist){
				closestInd = i;
				closestDist = dist;
			};
		}

		map.getSource('remaining_path').setData(LineString(visual_path_coordinates.slice(closestInd)));
	
		map.flyTo({
			center:[x1,y1],
			zoom: 19,
		})
	}
});

//graph_visual.subscribe(function({markers}) {
//
//	let arrows = markers.filter(({type})=>type==0);
//	map.addSource('graph_visual', {
//		'type':'geojson',
//		'data':{
//			'type':'Feature',
//			'geometry':{
//				'type':'MultiLineString',
//				'coordinates': arrows.map(({points})=>points.map(T))
//			}
//		}
//	});
//	map.addLayer({
//		'id':'graph_visual',
//		'type':'line',
//		'source':'graph_visual',
//		'layout': {
//                'line-join': 'round',
//                'line-cap': 'round'
//            },
//            'paint': {
//                'line-color': '#000',
//		'line-opacity': 0.03,
//                "line-width": {
//          "base": 1.4,
//          "stops": [
//            [14, 0],
//            [20, 18]
//          ]
//        }
//            }
//	});
//
//});
map.addLayer({
            'id': 'limited_pose',
            'type': 'symbol',
            'source': 'limited_pose',
            'layout': {
                'icon-image': 'custom-marker',
            }
        });
//// When a click event occurs on a feature in the places layer, open a popup at the
//        // location of the feature, with description HTML from its properties.
//        map.on('click', 'graph_visual', (e) => {
//		const {lat,lng} = e.lngLat;
//		navigateTo(lat,lng);
//	    });
//
//        // Change the cursor to a pointer when the mouse is over the places layer.
//        map.on('mouseenter', 'graph_visual', () => {
//            map.getCanvas().style.cursor = 'pointer';
//        });
//
//        // Change it back to a pointer when it leaves.
//        map.on('mouseleave', 'graph_visual', () => {
//            map.getCanvas().style.cursor = '';
//        });
});
