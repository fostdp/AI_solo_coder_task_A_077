window.Sat3DViewer = (function() {
  var API_BASE = 'http://localhost:8080';
  var WS_URL = 'ws://localhost:8080/ws';
  var RE_EARTH = 6378.137;

  var scene, camera, renderer, controls, raycaster, mouse;
  var earthGroup, atmosphereMesh, satelliteGroup, orbitGroup, encounterGroup;
  var satBodyInstanced, leftPanelInstanced, rightPanelInstanced, dummyLeft, dummyRight;
  var satelliteIndexMap = {};
  var satelliteDataMap = {};
  var encounterMarkers = [];
  var encounterLines = [];
  var clock = new THREE.Clock();
  var ws = null;
  var wsReconnectTimer = null;
  var onSatelliteClickCallback = null;

  function scaleKm(km) { return km / RE_EARTH; }

  function riskColor(level) {
    if (level === 'danger') return 0xff0000;
    if (level === 'warning') return 0xffff00;
    return 0x00ff00;
  }

  function initScene() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
    camera.position.set(0, 2, 3);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x0a0a2e, 1);
    document.getElementById('canvas-container').appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 1.5;
    controls.maxDistance = 10;
    controls.target.set(0, 0, 0);

    raycaster = new THREE.Raycaster();
    mouse = new THREE.Vector2();

    var ambientLight = new THREE.AmbientLight(0x334466, 0.6);
    scene.add(ambientLight);

    var dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
    dirLight.position.set(5, 3, 5);
    scene.add(dirLight);

    var dirLight2 = new THREE.DirectionalLight(0x4466aa, 0.3);
    dirLight2.position.set(-3, -1, -3);
    scene.add(dirLight2);

    createStars();
    createEarth();
    createGroups();

    window.addEventListener('resize', onWindowResize);
    renderer.domElement.addEventListener('click', onCanvasClick);
  }

  function createStars() {
    var geo = new THREE.BufferGeometry();
    var count = 3000;
    var positions = new Float32Array(count * 3);
    for (var i = 0; i < count * 3; i++) {
      positions[i] = (Math.random() - 0.5) * 60;
    }
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    var mat = new THREE.PointsMaterial({ color: 0xffffff, size: 0.03, sizeAttenuation: true });
    scene.add(new THREE.Points(geo, mat));
  }

  function createEarth() {
    earthGroup = new THREE.Group();
    scene.add(earthGroup);

    var earthGeo = new THREE.SphereGeometry(1, 64, 48);
    var earthMat = new THREE.MeshPhongMaterial({
      color: 0x1a4a8a,
      emissive: 0x0a1a3a,
      specular: 0x333366,
      shininess: 15
    });
    var earthMesh = new THREE.Mesh(earthGeo, earthMat);
    earthGroup.add(earthMesh);

    var atmosGeo = new THREE.SphereGeometry(1.02, 64, 48);
    var atmosMat = new THREE.MeshPhongMaterial({
      color: 0x4488ff,
      transparent: true,
      opacity: 0.08,
      side: THREE.BackSide
    });
    atmosphereMesh = new THREE.Mesh(atmosGeo, atmosMat);
    earthGroup.add(atmosphereMesh);

    var glowGeo = new THREE.SphereGeometry(1.06, 32, 24);
    var glowMat = new THREE.MeshBasicMaterial({
      color: 0x3366ff,
      transparent: true,
      opacity: 0.04,
      side: THREE.BackSide
    });
    earthGroup.add(new THREE.Mesh(glowGeo, glowMat));

    createGridLines();
  }

  function createGridLines() {
    var gridMat = new THREE.LineBasicMaterial({ color: 0x3366aa, transparent: true, opacity: 0.15 });
    var r = 1.001;

    for (var lat = -80; lat <= 80; lat += 20) {
      var phi = (90 - lat) * Math.PI / 180;
      var pts = [];
      for (var lon = 0; lon <= 360; lon += 3) {
        var theta = lon * Math.PI / 180;
        pts.push(new THREE.Vector3(
          r * Math.sin(phi) * Math.cos(theta),
          r * Math.cos(phi),
          r * Math.sin(phi) * Math.sin(theta)
        ));
      }
      var geo = new THREE.BufferGeometry().setFromPoints(pts);
      earthGroup.add(new THREE.Line(geo, gridMat));
    }

    for (var lon2 = 0; lon2 < 360; lon2 += 30) {
      var theta2 = lon2 * Math.PI / 180;
      var pts2 = [];
      for (var lat2 = -90; lat2 <= 90; lat2 += 3) {
        var phi2 = (90 - lat2) * Math.PI / 180;
        pts2.push(new THREE.Vector3(
          r * Math.sin(phi2) * Math.cos(theta2),
          r * Math.cos(phi2),
          r * Math.sin(phi2) * Math.sin(theta2)
        ));
      }
      var geo2 = new THREE.BufferGeometry().setFromPoints(pts2);
      earthGroup.add(new THREE.Line(geo2, gridMat));
    }
  }

  function createGroups() {
    orbitGroup = new THREE.Group();
    scene.add(orbitGroup);
    satelliteGroup = new THREE.Group();
    scene.add(satelliteGroup);
    encounterGroup = new THREE.Group();
    scene.add(encounterGroup);

    var MAX_SATELLITES = 80;
    var satBodyGeo = new THREE.BoxGeometry(0.015, 0.01, 0.01);
    var satBodyMat = new THREE.MeshPhongMaterial({ color: 0x00ff00, emissive: 0x00ff00, emissiveIntensity: 0.3 });
    satBodyInstanced = new THREE.InstancedMesh(satBodyGeo, satBodyMat, MAX_SATELLITES);
    satBodyInstanced.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    satBodyInstanced.count = 0;
    satelliteGroup.add(satBodyInstanced);

    var panelGeo = new THREE.BoxGeometry(0.025, 0.001, 0.008);
    var panelMat = new THREE.MeshPhongMaterial({ color: 0x2244aa, emissive: 0x112244, emissiveIntensity: 0.2 });
    leftPanelInstanced = new THREE.InstancedMesh(panelGeo, panelMat, MAX_SATELLITES);
    leftPanelInstanced.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    leftPanelInstanced.count = 0;
    rightPanelInstanced = new THREE.InstancedMesh(panelGeo, panelMat, MAX_SATELLITES);
    rightPanelInstanced.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    rightPanelInstanced.count = 0;
    satelliteGroup.add(leftPanelInstanced);
    satelliteGroup.add(rightPanelInstanced);

    dummyLeft = new THREE.Object3D();
    dummyRight = new THREE.Object3D();
  }

  function updateSatellites(satellites) {
    var count = Math.min(satellites.length, 80);
    satBodyInstanced.count = count;
    leftPanelInstanced.count = count;
    rightPanelInstanced.count = count;

    var dummy = new THREE.Object3D();
    var color = new THREE.Color();

    for (var i = 0; i < count; i++) {
      var sat = satellites[i];
      var pos = sat.current_position;
      satelliteDataMap[sat.satellite_id] = sat;
      satelliteIndexMap[sat.satellite_id] = i;

      dummy.position.set(scaleKm(pos.x), scaleKm(pos.y), scaleKm(pos.z));
      dummy.updateMatrix();
      satBodyInstanced.setMatrixAt(i, dummy.matrix);

      color.setHex(riskColor(sat.collision_risk_level));
      satBodyInstanced.setColorAt(i, color);

      dummyLeft.position.set(scaleKm(pos.x) - 0.02, scaleKm(pos.y), scaleKm(pos.z));
      dummyLeft.updateMatrix();
      leftPanelInstanced.setMatrixAt(i, dummyLeft.matrix);

      dummyRight.position.set(scaleKm(pos.x) + 0.02, scaleKm(pos.y), scaleKm(pos.z));
      dummyRight.updateMatrix();
      rightPanelInstanced.setMatrixAt(i, dummyRight.matrix);
    }

    satBodyInstanced.instanceMatrix.needsUpdate = true;
    if (satBodyInstanced.instanceColor) satBodyInstanced.instanceColor.needsUpdate = true;
    leftPanelInstanced.instanceMatrix.needsUpdate = true;
    rightPanelInstanced.instanceMatrix.needsUpdate = true;
  }

  function updateSatellitePositionsFromWS(positions) {
    if (!positions || !Array.isArray(positions)) return;
    var dummy = new THREE.Object3D();
    var currentCount = satBodyInstanced.count;

    positions.forEach(function(p) {
      var idx = satelliteIndexMap[p.satellite_id];
      if (idx !== undefined && idx < currentCount && p.position) {
        dummy.position.set(scaleKm(p.position.x), scaleKm(p.position.y), scaleKm(p.position.z));
        dummy.updateMatrix();
        satBodyInstanced.setMatrixAt(idx, dummy.matrix);

        dummyLeft.position.set(scaleKm(p.position.x) - 0.02, scaleKm(p.position.y), scaleKm(p.position.z));
        dummyLeft.updateMatrix();
        leftPanelInstanced.setMatrixAt(idx, dummyLeft.matrix);

        dummyRight.position.set(scaleKm(p.position.x) + 0.02, scaleKm(p.position.y), scaleKm(p.position.z));
        dummyRight.updateMatrix();
        rightPanelInstanced.setMatrixAt(idx, dummyRight.matrix);
      }
    });

    satBodyInstanced.instanceMatrix.needsUpdate = true;
    leftPanelInstanced.instanceMatrix.needsUpdate = true;
    rightPanelInstanced.instanceMatrix.needsUpdate = true;
  }

  function loadOrbitPaths(satellites) {
    while (orbitGroup.children.length > 0) {
      orbitGroup.remove(orbitGroup.children[0]);
    }

    var camPos = camera.position;
    var camDistThreshold = 5.0;

    satellites.forEach(function(sat) {
      var id = sat.satellite_id;
      var risk = sat.collision_risk_level;
      var pos = sat.current_position;
      var satWorldPos = new THREE.Vector3(scaleKm(pos.x), scaleKm(pos.y), scaleKm(pos.z));
      var distToCam = camPos.distanceTo(satWorldPos);

      if (distToCam > camDistThreshold && risk === 'safe') return;

      var segments;
      if (risk === 'safe') {
        segments = 24;
      } else if (risk === 'warning') {
        segments = 48;
      } else {
        segments = 96;
      }

      fetch(API_BASE + '/api/satellites/' + id + '/orbit-path')
        .then(function(r) { return r.json(); })
        .then(function(path) {
          if (!Array.isArray(path) || path.length === 0) return;
          var step = Math.max(1, Math.floor(path.length / segments));
          var sampledPoints = [];
          for (var i = 0; i < path.length; i += step) {
            var p = path[i];
            sampledPoints.push(new THREE.Vector3(scaleKm(p.x), scaleKm(p.y), scaleKm(p.z)));
          }
          var lastP = path[path.length - 1];
          var lastVec = new THREE.Vector3(scaleKm(lastP.x), scaleKm(lastP.y), scaleKm(lastP.z));
          if (sampledPoints.length > 0 && !sampledPoints[sampledPoints.length - 1].equals(lastVec)) {
            sampledPoints.push(lastVec);
          }
          var geo = new THREE.BufferGeometry().setFromPoints(sampledPoints);
          var color = riskColor(risk);
          var mat = new THREE.LineBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.2
          });
          var line = new THREE.Line(geo, mat);
          orbitGroup.add(line);
        })
        .catch(function() {});
    });
  }

  function loadEncounterPoints(encounters) {
    while (encounterGroup.children.length > 0) {
      encounterGroup.remove(encounterGroup.children[0]);
    }
    encounterMarkers = [];
    encounterLines = [];

    if (!encounters || !Array.isArray(encounters)) return;

    var matrix = new THREE.Matrix4();

    encounters.forEach(function(enc) {
      if (enc.alert_level <= 0) return;

      var idx1 = satelliteIndexMap[enc.satellite_id_1];
      var idx2 = satelliteIndexMap[enc.satellite_id_2];
      if (idx1 === undefined || idx2 === undefined) return;

      var pos1 = new THREE.Vector3();
      var pos2 = new THREE.Vector3();

      if (idx1 < satBodyInstanced.count) {
        satBodyInstanced.getMatrixAt(idx1, matrix);
        pos1.setFromMatrixPosition(matrix);
      } else {
        return;
      }
      if (idx2 < satBodyInstanced.count) {
        satBodyInstanced.getMatrixAt(idx2, matrix);
        pos2.setFromMatrixPosition(matrix);
      } else {
        return;
      }

      var pts = [pos1.clone(), pos2.clone()];
      var geo = new THREE.BufferGeometry().setFromPoints(pts);
      var mat = new THREE.LineDashedMaterial({
        color: 0xff0000,
        dashSize: 0.02,
        gapSize: 0.01,
        transparent: true,
        opacity: 0.7
      });
      var line = new THREE.Line(geo, mat);
      line.computeLineDistances();
      encounterGroup.add(line);
      encounterLines.push(line);

      if (enc.encounter_point_eci) {
        var ep = enc.encounter_point_eci;
        var markerGeo = new THREE.SphereGeometry(0.012, 12, 8);
        var markerMat = new THREE.MeshBasicMaterial({
          color: 0xff0000,
          transparent: true,
          opacity: 0.8
        });
        var marker = new THREE.Mesh(markerGeo, markerMat);
        marker.position.set(scaleKm(ep[0]), scaleKm(ep[1]), scaleKm(ep[2]));
        encounterGroup.add(marker);
        encounterMarkers.push(marker);
      }
    });
  }

  function onCanvasClick(event) {
    var rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    var intersects = raycaster.intersectObject(satBodyInstanced);
    if (intersects.length > 0) {
      var instanceId = intersects[0].instanceId;
      var satId = Object.keys(satelliteIndexMap).find(function(key) {
        return satelliteIndexMap[key] === instanceId;
      });
      if (satId !== undefined) {
        if (onSatelliteClickCallback) {
          onSatelliteClickCallback(parseInt(satId));
        }
      }
    }
  }

  function connectWebSocket() {
    var statusEl = document.getElementById('ws-status');
    try {
      ws = new WebSocket(WS_URL);
    } catch (e) {
      statusEl.textContent = 'WS: 连接失败';
      statusEl.className = 'ws-status disconnected';
      scheduleReconnect();
      return;
    }

    ws.onopen = function() {
      statusEl.textContent = 'WS: 已连接';
      statusEl.className = 'ws-status connected';
    };

    ws.onmessage = function(event) {
      try {
        var data = JSON.parse(event.data);
        if (Array.isArray(data)) {
          updateSatellitePositionsFromWS(data);
        }
      } catch (e) {
        console.error('WS parse error:', e);
      }
    };

    ws.onclose = function() {
      statusEl.textContent = 'WS: 已断开';
      statusEl.className = 'ws-status disconnected';
      scheduleReconnect();
    };

    ws.onerror = function() {
      statusEl.textContent = 'WS: 连接错误';
      statusEl.className = 'ws-status disconnected';
    };
  }

  function scheduleReconnect() {
    if (wsReconnectTimer) clearTimeout(wsReconnectTimer);
    wsReconnectTimer = setTimeout(function() {
      connectWebSocket();
    }, 5000);
  }

  function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  }

  function animate() {
    requestAnimationFrame(animate);
    var delta = clock.getDelta();
    var elapsed = clock.getElapsedTime();

    controls.update();

    if (earthGroup) {
      earthGroup.rotation.y += delta * 0.02;
    }

    encounterMarkers.forEach(function(marker, idx) {
      var pulse = 0.6 + 0.4 * Math.sin(elapsed * 3 + idx);
      marker.material.opacity = pulse;
      marker.scale.setScalar(0.8 + 0.4 * Math.sin(elapsed * 3 + idx));
    });

    if (satBodyInstanced.count > 0) {
      var blinkColor = new THREE.Color();
      for (var id in satelliteDataMap) {
        var sat = satelliteDataMap[id];
        if (sat.collision_risk_level === 'danger') {
          var idx = satelliteIndexMap[id];
          if (idx !== undefined) {
            blinkColor.setHex(0xff0000);
            blinkColor.multiplyScalar(0.7 + 0.3 * Math.sin(elapsed * 5));
            satBodyInstanced.setColorAt(idx, blinkColor);
          }
        }
      }
      if (satBodyInstanced.instanceColor) satBodyInstanced.instanceColor.needsUpdate = true;
    }

    renderer.render(scene, camera);
  }

  function loadAllData() {
    fetch(API_BASE + '/api/satellites')
      .then(function(r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      })
      .then(function(satellites) {
        updateSatellites(satellites);
        loadOrbitPaths(satellites);

        return fetch(API_BASE + '/api/collision-encounters')
          .then(function(r) { return r.json(); })
          .then(function(encounters) {
            loadEncounterPoints(encounters);
          })
          .catch(function() {});
      })
      .catch(function(err) {
        console.error('Failed to load satellite data:', err);
      });
  }

  function startAutoRefresh() {
    setInterval(function() {
      fetch(API_BASE + '/api/collision-encounters')
        .then(function(r) { return r.json(); })
        .then(function(encounters) {
          loadEncounterPoints(encounters);
        })
        .catch(function() {});
    }, 30000);
  }

  function init() {
    initScene();
    loadAllData();
    connectWebSocket();
    startAutoRefresh();
    animate();
  }

  return {
    init: init,
    satelliteIndexMap: satelliteIndexMap,
    satelliteDataMap: satelliteDataMap,
    updateSatellites: updateSatellites,
    loadOrbitPaths: loadOrbitPaths,
    loadEncounterPoints: loadEncounterPoints,
    setOnSatelliteClick: function(cb) { onSatelliteClickCallback = cb; },
    getSatData: function(id) { return satelliteDataMap[id]; }
  };
})();
