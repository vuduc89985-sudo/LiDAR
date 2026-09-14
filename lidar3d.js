import * as THREE from '../vendor/three.module.js';
import { OrbitControls } from '../vendor/OrbitControls.js';

export class Lidar3D {
  constructor(host, panel, palette) {
    this.host = host;
    this.panel = panel;
    this.palette = palette;
    this.layers = new Map();
    this.enabled = false;
    this.fusion = new LidarProtocol.FusionFrames();
    this.pointSize = 3;
    this.decimation = 1;
    this.showPlanes = true;

    // Quản lý Zone từ Code 1
    this.zones = [];
    this.zoneExtrinsicSignature = '';
    this.zoneRenderOffset = 0.002;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#f5f8fc');
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 2000);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    
    this.renderer.domElement.style.pointerEvents = 'auto';
    this.renderer.domElement.style.position = 'absolute';
    this.renderer.domElement.style.top = '0';
    this.renderer.domElement.style.left = '0';
    this.renderer.domElement.style.zIndex = '1';
    host.prepend(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.baseGridSize = 100;
    this.grid = new THREE.GridHelper(this.baseGridSize, this.baseGridSize, 0x6b8099, 0xcbd5e1);
    this.grid.rotation.x = Math.PI / 2;
    this.scene.add(this.grid);

    this.baseAxesSize = 15;
    this.axes = new THREE.AxesHelper(this.baseAxesSize);
    this.scene.add(this.axes);

        // Group quản lý Zone
    this.zoneGroup = new THREE.Group();
    this.zoneGroup.name = 'zones';
    this.scene.add(this.zoneGroup);

    // === Muc tieu chuyen dong (targets) + canh bao 3D ===
    this.targetGroup = new THREE.Group();
    this.targetGroup.name = 'targets';
    this.scene.add(this.targetGroup);
    this.targetMeshes = new Map(); // targetId -> { group, fill, edges, label }
    this.alarmActive = false;

    this.alarmBanner = document.createElement('div');
    Object.assign(this.alarmBanner.style, {
      position: 'absolute', top: '8px', left: '50%', transform: 'translateX(-50%)',
      background: '#c00', color: '#fff', fontWeight: 'bold', padding: '6px 16px',
      borderRadius: '4px', zIndex: 6, display: 'none', fontSize: '14px',
      boxShadow: '0 2px 8px rgba(0,0,0,.4)'
    });
    this.alarmBanner.textContent = '⚠ CẢNH BÁO: PHÁT HIỆN CHUYỂN ĐỘNG TRONG VÙNG';
    host.appendChild(this.alarmBanner);

    this.axisLabels = {};
    for (const [name, pos, color] of [['X', [3.3, 0, 0], '#d22'], ['Y', [0, 3.3, 0], '#292'], ['Z', [0, 0, 3.3], '#22d']]) {
      const label = this.label(name + ' (m)', color);
      label.position.set(...pos);
      this.scene.add(label);
      this.axisLabels[name] = label;
    }

    this.preset('Perspective');
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(host);

    this.distanceInfoAllEl = document.getElementById('distanceInfoAllPanel') || document.getElementById('distanceInfoAll');
    this.distanceUpdateTimeEl = document.getElementById('distanceUpdateTime');

    // Raycaster cho tương tác 3D
    this.raycaster = new THREE.Raycaster();
    this.mouse = new THREE.Vector2();
    this.groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    
    this.zoneTemp = null;
    this.zoneTempVertices = [];
    this.pickRef = [];
    this.pickTarget = [];
    this.rulerPoints = [];

    this.renderer.domElement.addEventListener('click', (e) => this.onCanvasClick(e));
    this.renderer.domElement.addEventListener('contextmenu', (e) => this.onCanvasRightClick(e));
  }

  // === CÁC HÀM XỬ LÝ ZONE 3D TỪ CODE 1 ===
  extrinsicFor(sourceId, extrinsics) {
    return extrinsics.find(e => e?.id === sourceId) || extrinsics[sourceId] || {};
  }

  zoneExtrinsic(e) {
    return {
      ox: +(e.x ?? e.dx) || 0,
      oy: +(e.y ?? e.dy) || 0,
      oz: +e.z || 0,
      roll: (+e.roll || 0) * Math.PI / 180,
      pitch: (+e.pitch || 0) * Math.PI / 180,
      oth: (+(e.yaw ?? e.thetaDeg) || 0) * Math.PI / 180
    };
  }

  zonePoints(zone) {
    if (zone.type === 0) return [[zone.x1, zone.y1], [zone.x2, zone.y1], [zone.x2, zone.y2], [zone.x1, zone.y2]];
    if (zone.type === 1 || zone.type === 4) return zone.vertices || [];
    if (zone.type === 2) return Array.from({ length: 64 }, (_, i) => {
      const a = i * Math.PI * 2 / 64;
      return [zone.cx + zone.radius * Math.cos(a), zone.cy + zone.radius * Math.sin(a)];
    });
    if (zone.type === 3) {
      const points = [[zone.sx, zone.sy]], a1 = zone.sAngleStart * Math.PI / 180, a2 = zone.sAngleEnd * Math.PI / 180;
      const span = ((a2 - a1 + Math.PI * 2) % (Math.PI * 2)) || Math.PI * 2;
      for (let i = 0; i <= 48; i++) {
        const a = a1 + span * i / 48;
        points.push([zone.sx + zone.sRadius * Math.cos(a), zone.sy + zone.sRadius * Math.sin(a)]);
      }
      return points;
    }
    return [];
  }

  clearZones() {
    for (const child of [...this.zoneGroup.children]) {
      this.zoneGroup.remove(child);
      this.disposeObject(child);
    }
  }

  setZones(zones, extrinsics = []) {
    this.zones = zones.map(z => ({ ...z, vertices: z.vertices?.map(p => [+p[0], +p[1]]) }));
    this.rebuildZones(extrinsics);
  }

  rebuildZones(extrinsics = []) {
    this.clearZones();
    for (const zone of this.zones) {
      const local = zone.scope === 'local';
      const rawSourceId = Number(zone.sourceId ?? zone.panel ?? zone.li ?? 0);
      const sourceId = Number.isInteger(rawSourceId) && rawSourceId >= 0 ? rawSourceId : 0;
      const e = local ? this.zoneExtrinsic(this.extrinsicFor(sourceId, extrinsics)) : null;
      const r = local ? LidarProtocol.rotation(e) : null;
      const normal = local ? [r[2], r[5], r[8]] : [0, 0, 1];

      const points = this.zonePoints(zone).map(([x, y]) => {
        const p = local ? LidarProtocol.localPointToWorld3D(e, +x, +y, 0) : [+x, +y, 0];
        return new THREE.Vector3(
          p[0] + normal[0] * this.zoneRenderOffset,
          p[1] + normal[1] * this.zoneRenderOffset,
          p[2] + normal[2] * this.zoneRenderOffset
        );
      });

      if (points.length < 2 || points.some(p => !Number.isFinite(p.x + p.y + p.z))) continue;
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const color = local ? this.palette[((sourceId % this.palette.length) + this.palette.length) % this.palette.length] : '#d62728';
      const material = new THREE.LineBasicMaterial({ color, depthTest: true, transparent: true, opacity: 0.95 });
      const line = new THREE.LineLoop(geometry, material);
      line.userData = { zoneId: zone.id, scope: zone.scope, sourceId: local ? sourceId : null };
      line.renderOrder = 2;
      this.zoneGroup.add(line);
    }
  }

  // === TƯƠNG TÁC CANVAS (RULER / PICK / ZONE) ===
  onCanvasClick(event) {
    if (event.target !== this.renderer.domElement) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.mouse, this.camera);
    const intersectPoint = new THREE.Vector3();
    this.raycaster.ray.intersectPlane(this.groundPlane, intersectPoint);
    if (!intersectPoint) return;
    const wx = intersectPoint.x;
    const wy = intersectPoint.y;
    if (window.rulerMode) {
      this.handleRuler(wx, wy);
    } else if (window.pickMode) {
      this.handlePick(wx, wy);
    } else if (window.zoneMode && window.zoneTool) {
      this.handleZone(wx, wy);
    }
  }

  onCanvasRightClick(event) {
    event.preventDefault();
    if (window.zoneMode && (window.zoneTool === 'polygon' || window.zoneTool === 'freehand')) {
      this.finishZone();
    }
  }

  handleRuler(wx, wy) {
    this.rulerPoints.push([wx, wy]);
    if (this.rulerPoints.length === 2) {
      const p1 = this.rulerPoints[0];
      const p2 = this.rulerPoints[1];
      const dist = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
      alert(`Khoảng cách: ${dist.toFixed(2)} m`);
      this.rulerPoints = [];
    }
  }

  handlePick(wx, wy) {
    if (this.pickRef.length < 2) {
      this.pickRef.push({ x: wx, y: wy });
    } else if (this.pickTarget.length < 2) {
      this.pickTarget.push({ x: wx, y: wy });
    }
    const btn = document.getElementById('btnPick');
    if (btn) btn.textContent = `Pick R:${this.pickRef.length} T:${this.pickTarget.length}`;
    if (this.pickRef.length === 2 && this.pickTarget.length === 2) {
      if (window.send) {
        window.send({ cmd: 'pick_apply', ref: this.pickRef, target: this.pickTarget });
      }
      this.pickRef = [];
      this.pickTarget = [];
      if (btn) {
        btn.textContent = 'Pick R:0 T:0';
        window.pickMode = false;
        btn.classList.remove('active');
      }
    }
  }

  handleZone(wx, wy) {
    const scope = document.getElementById('zoneScope')?.value || 'fusion';
    const tool = window.zoneTool;
    if (tool === 'rect') {
      if (!this.zoneTemp) {
        this.zoneTemp = { x1: wx, y1: wy };
      } else {
        const zone = {
          type: 0, scope, id: Date.now(),
          x1: Math.min(this.zoneTemp.x1, wx), y1: Math.min(this.zoneTemp.y1, wy),
          x2: Math.max(this.zoneTemp.x1, wx), y2: Math.max(this.zoneTemp.y1, wy)
        };
        this.sendZoneToBackend(zone);
        this.zoneTemp = null;
      }
    } else if (tool === 'circle') {
      if (!this.zoneTemp) {
        this.zoneTemp = { cx: wx, cy: wy };
      } else {
        const radius = Math.hypot(wx - this.zoneTemp.cx, wy - this.zoneTemp.cy);
        const zone = { type: 2, scope, id: Date.now(), cx: this.zoneTemp.cx, cy: this.zoneTemp.cy, radius };
        this.sendZoneToBackend(zone);
        this.zoneTemp = null;
        window.zoneMode = false;
        document.getElementById('btnZone')?.classList.remove('active');
      }
    } else if (tool === 'sector') {
      if (!this.zoneTemp) {
        this.zoneTemp = { sx: wx, sy: wy, step: 0 };
      } else if (this.zoneTemp.step === 0) {
        this.zoneTemp.radius = Math.hypot(wx - this.zoneTemp.sx, wy - this.zoneTemp.sy);
        this.zoneTemp.angleStart = Math.atan2(wy - this.zoneTemp.sy, wx - this.zoneTemp.sx) * 180 / Math.PI;
        this.zoneTemp.step = 1;
      } else {
        const angleEnd = Math.atan2(wy - this.zoneTemp.sy, wx - this.zoneTemp.sx) * 180 / Math.PI;
        const zone = {
          type: 3, scope, id: Date.now(),
          sx: this.zoneTemp.sx, sy: this.zoneTemp.sy,
          sRadius: this.zoneTemp.radius,
          sAngleStart: this.zoneTemp.angleStart, sAngleEnd: angleEnd
        };
        this.sendZoneToBackend(zone);
        this.zoneTemp = null;
        window.zoneMode = false;
        document.getElementById('btnZone')?.classList.remove('active');
      }
    } else if (tool === 'polygon' || tool === 'freehand') {
      this.zoneTempVertices.push([wx, wy]);
      console.log(`[Zone 3D] Thêm đỉnh: ${wx.toFixed(2)}, ${wy.toFixed(2)}`);
    }
  }

  finishZone() {
    if ((window.zoneTool === 'polygon' || window.zoneTool === 'freehand') && this.zoneTempVertices.length >= 3) {
      const scope = document.getElementById('zoneScope')?.value || 'fusion';
      const zone = {
        type: window.zoneTool === 'freehand' ? 4 : 1,
        scope, id: Date.now(),
        vertices: this.zoneTempVertices
      };
      this.sendZoneToBackend(zone);
    }
    this.zoneTempVertices = [];
    window.zoneMode = false;
    window.zoneTool = null;
    document.getElementById('btnZone')?.classList.remove('active');
    document.querySelectorAll('#zoneToolbar button').forEach(b => b.classList.remove('active'));
  }

  sendZoneToBackend(newZone) {
    window.zones = window.zones || [];
    newZone.id = window.zoneNextId++;
    window.zones.push(newZone);
    const arr = window.zones.map(z => {
      const o = { scope: z.scope || 'fusion', id: z.id, type: z.type };
      if (z.type === 0) { o.x1 = z.x1; o.y1 = z.y1; o.x2 = z.x2; o.y2 = z.y2; }
      else if (z.type === 1 || z.type === 4) { o.vertices = z.vertices; }
      else if (z.type === 2) { o.cx = z.cx; o.cy = z.cy; o.radius = z.radius; }
      else { o.sx = z.sx; o.sy = z.sy; o.sRadius = z.sRadius; o.sAngleStart = z.sAngleStart; o.sAngleEnd = z.sAngleEnd; }
      return o;
    });
    if (window.send) {
      window.send({ cmd: 'zones_set', zones: arr });
    }
    console.log('[Zone 3D] Đã gửi zone lên backend. Tổng zones:', window.zones.length);
  }

  label(text, color) {
    const c = document.createElement('canvas');
    c.width = 512; c.height = 64;
    const g = c.getContext('2d');
    g.font = '28px sans-serif';
    g.fillStyle = color;
    g.fillText(text.slice(0, 32), 8, 42);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
    sprite.scale.set(2.8, 0.35, 1);
    return sprite;
  }

  preset(name) {
    const pos = { Top: [0, 0, 80], Front: [0, -80, 1], Side: [80, 0, 1], Perspective: [50, -50, 50] }[name] || [50, -50, 50];
    this.camera.up.set(...(name === 'Top' ? [0, 1, 0] : [0, 0, 1]));
    this.controls.target.set(0, 0, 1);
    this.camera.position.set(...pos);
    this.controls.update();
  }

  resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  makeLayer(id, name) {
    const color = this.palette[id % this.palette.length];
    const group = new THREE.Group();
    this.scene.add(group);

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    const points = new THREE.Points(geometry, new THREE.PointsMaterial({ 
      color, 
      size: this.pointSize, 
      sizeAttenuation: false,
      transparent: true,
      opacity: 1.0
    }));
    points.frustumCulled = false;
    group.add(points);

    const sensor = new THREE.Group();
    group.add(sensor);
    sensor.add(new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 8), new THREE.MeshBasicMaterial({ color })));
    sensor.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.8, color));

    const scanRadius = 15;
    const thetaStart = -Math.PI / 4;
    const thetaLength = 3 * Math.PI / 2;

    const scanZone = new THREE.Mesh(
      new THREE.RingGeometry(0, scanRadius, 64, 1, thetaStart, thetaLength),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.15, side: THREE.DoubleSide, depthWrite: false })
    );
    sensor.add(scanZone);

    const blindStart = thetaStart + thetaLength;
    const blindLength = 2 * Math.PI - thetaLength;
    const blindZone = new THREE.Mesh(
      new THREE.RingGeometry(scanRadius * 0.9, scanRadius, 16, 1, blindStart, blindLength),
      new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false })
    );
    sensor.add(blindZone);

    const label = this.label(name, color);
    group.add(label);

    const row = document.createElement('div'),
          check = document.createElement('input'),
          text = document.createElement('span');
    check.type = 'checkbox';
    check.checked = true;
    row.append(check, text);
    this.panel.append(row);

    const layer = { group, points, sensor, scanZone, blindZone, label, row, check, text, name };
    this.layers.set(id, layer);
    return layer;
  }

  disposeObject(obj) {
    obj.traverse(o => {
      o.geometry?.dispose();
      if (o.material) {
        for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
          m.map?.dispose();
          m.dispose();
        }
      }
    });
  }

  removeLayer(id) {
    const l = this.layers.get(id);
    if (!l) return;
    this.scene.remove(l.group);
    this.disposeObject(l.group);
    l.row.remove();
    this.layers.delete(id);
  }

  clear() {
    this.fusion.clear();
    for (const id of [...this.layers.keys()]) this.removeLayer(id);
  }

  updateDistanceInfoAll() {
    if (!this.distanceInfoAllEl) return;
    const cameraPos = this.camera.position;
    const now = performance.now();
    let html = '';
    let hasAnyData = false;
    let totalOnline = 0;
    let totalPoints = 0;

    for (const [id, layer] of this.layers) {
      hasAnyData = true;
      const color = this.palette[id % this.palette.length];
      const name = layer.name || `LiDAR ${id + 1}`;
      const sensorPos = new THREE.Vector3();
      layer.sensor.getWorldPosition(sensorPos);
      const camDist = cameraPos.distanceTo(sensorPos);
      const frame = this.fusion.hist.get(id)?.at(-1);

      let status = 'offline', statusColor = '#ff6b6b', statusIcon = '●', pts = 0;
      let rMin = 'N/A', rMax = 'N/A', scanAngle = 'N/A', lastUpdate = '--';

      if (frame) {
        const age = now - frame.recvTime;
        if (age < 1000) {
          status = 'online'; statusColor = '#51cf66'; statusIcon = '●';
          pts = frame.n || 0; totalPoints += pts; totalOnline++;
          if (typeof frame.rMin === 'number' && isFinite(frame.rMin)) rMin = frame.rMin.toFixed(2) + ' m';
          if (typeof frame.rMax === 'number' && isFinite(frame.rMax)) rMax = frame.rMax.toFixed(2) + ' m';
          if (typeof frame.aMin === 'number' && typeof frame.aMax === 'number') {
            const aMinDeg = (frame.aMin * 180 / Math.PI).toFixed(1);
            const aMaxDeg = (frame.aMax * 180 / Math.PI).toFixed(1);
            scanAngle = `${((frame.aMax - frame.aMin) * 180 / Math.PI).toFixed(1)}° (${aMinDeg}°→${aMaxDeg}°)`;
          }
          const ageMs = Math.round(age);
          lastUpdate = ageMs < 100 ? 'vừa xong' : `${ageMs} ms trước`;
        } else {
          status = 'stale'; statusColor = '#ffd43b'; statusIcon = '◐';
          pts = frame.n || 0; lastUpdate = `${Math.round(age)} ms trước`;
        }
      }

      let distToFirst = '';
      if (id > 0 && this.layers.has(0)) {
        const firstPos = new THREE.Vector3();
        this.layers.get(0).sensor.getWorldPosition(firstPos);
        distToFirst = sensorPos.distanceTo(firstPos).toFixed(2) + ' m';
      }

      html += `<div style="background: rgba(30, 41, 59, 0.7); border-left: 3px solid ${color}; border-radius: 4px; padding: 8px; margin-bottom: 6px; font-size: 11px; backdrop-filter: blur(4px);">
          <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; padding-bottom: 4px; border-bottom: 1px solid rgba(148, 163, 184, 0.2);">
              <strong style="color: ${color}; font-size: 12px;"> ${name}</strong>
              <span style="color: ${statusColor}; font-size: 10px; font-weight: 600; text-transform: uppercase;">${statusIcon} ${status}</span>
          </div>
          <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 4px 8px;">
              <div><span style="color: #94a3b8;">📍 Vị trí (X,Y,Z):</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${sensorPos.x.toFixed(2)}, ${sensorPos.y.toFixed(2)}, ${sensorPos.z.toFixed(2)} m</span></div>
              <div><span style="color: #94a3b8;"> Cam → LiDAR:</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${camDist.toFixed(2)} m</span></div>
              <div><span style="color: #94a3b8;"> Gần nhất:</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${rMin}</span></div>
              <div><span style="color: #94a3b8;">🎯 Xa nhất:</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${rMax}</span></div>
              <div style="grid-column: span 2;"><span style="color: #94a3b8;">📐 Góc quét:</span><span style="color: #e2e8f0; font-family: monospace; font-size: 10px; margin-left: 4px;">${scanAngle}</span></div>
              <div><span style="color: #94a3b8;"> Điểm/frame:</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${pts.toLocaleString()}</span></div>
              <div><span style="color: #94a3b8;">🕐 Cập nhật:</span><br><span style="color: #e2e8f0; font-family: monospace; font-size: 10px;">${lastUpdate}</span></div>
              ${distToFirst ? `<div style="grid-column: span 2;"><span style="color: #94a3b8;">🔗 Cách LiDAR #1:</span><span style="color: #e2e8f0; font-family: monospace; font-size: 10px; margin-left: 4px;">${distToFirst}</span></div>` : ''}
          </div></div>`;
    }

    if (hasAnyData) {
      html = `<div style="background: rgba(59, 130, 246, 0.15); border: 1px solid rgba(59, 130, 246, 0.3); border-radius: 4px; padding: 6px 8px; margin-bottom: 8px; font-size: 11px;">
          <div style="display: flex; justify-content: space-between; align-items: center;">
              <span style="color: #93c5fd; font-weight: 600;">📊 TỔNG HỆ THỐNG</span>
              <span style="color: ${totalOnline > 0 ? '#51cf66' : '#ff6b6b'}; font-weight: 600;">${totalOnline}/${this.layers.size} online</span>
          </div>
          <div style="color: #e2e8f0; font-family: monospace; font-size: 10px; margin-top: 4px;">Tổng điểm: <strong>${totalPoints.toLocaleString()}</strong> pts/frame</div>
      </div>` + html;
    } else {
      html = '<div style="color:#94a3b8;padding:8px;text-align:center;">Đang chờ kết nối LiDAR...</div>';
    }

    this.distanceInfoAllEl.innerHTML = html;
    if (this.distanceUpdateTimeEl) {
      this.distanceUpdateTimeEl.textContent = new Date().toLocaleTimeString('vi-VN', { hour12: false });
    }
  }

  updateTargets(ev) {
    const targets = (ev && ev.targets) || [];
    const seen = new Set();
    
    for (const t of targets) {
      if (t.age < 3) continue; // giong nguong PERSISTENCE_FRAMES ben C++, bo qua khi chua xac nhan
      seen.add(t.id);

      const confirmed = t.age >= 3;
      const color = confirmed ? 0xff2020 : 0xffa500;

      let entry = this.targetMeshes.get(t.id);
      if (!entry) {
        const geometry = new THREE.BoxGeometry(1, 1, 1);
        const edges = new THREE.LineSegments(
          new THREE.EdgesGeometry(geometry),
          new THREE.LineBasicMaterial({ color, linewidth: 2 })
        );
        const fill = new THREE.Mesh(geometry,
          new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.15 }));
        const group = new THREE.Group();
        group.add(fill); group.add(edges);
        const label = this.label('#' + t.id, '#c00');
        this.targetGroup.add(group);
        this.targetGroup.add(label); // label khong bi anh huong boi scale cua group
        entry = { group, fill, edges, label };
        this.targetMeshes.set(t.id, entry);
      }

      // t.x, t.y la toa do THE GIOI - dung truc tiep vi scene 3D la khung Fusion
            // t.x, t.y, t.z1/t.z2 la toa do THE GIOI thuc te (backend tinh tu diem that,
      // tu dong dung theo do cao lap dat cua LiDAR, khong con hardcode)
      const zLo = Number.isFinite(t.z1) ? t.z1 : 0;
      const zHi = Number.isFinite(t.z2) ? t.z2 : zLo + 1.6;
      const h3 = Math.max(zHi - zLo, 0.3); // toi thieu 0.3m de luon thay ro khung
      entry.group.position.set(t.x, t.y, (zLo + zHi) / 2);
      entry.group.scale.set(Math.max(t.w, 0.1), Math.max(t.h, 0.1), h3);
      entry.label.position.set(t.x, t.y, zHi + 0.3);
      entry.fill.material.color.setHex(color);
      entry.edges.material.color.setHex(color);
    }

    for (const [id, entry] of this.targetMeshes) {
      if (!seen.has(id)) {
        this.targetGroup.remove(entry.group);
        this.targetGroup.remove(entry.label);
        this.disposeObject(entry.group);
        this.disposeObject(entry.label);
        this.targetMeshes.delete(id);
      }
    }

    const confirmedCount = targets.filter(t => t.age >= 3).length;
    this.alarmActive = confirmedCount > 0;
    this.alarmBanner.style.display = this.alarmActive ? 'block' : 'none';
  }

    update(names, extrinsics, paused, ev) {
    if (!this.enabled) return;

    // Kiếm tra sự thay đổi Extrinsics để tự động vẽ lại Zone (Nhận từ Code 1)
    const zoneSig = JSON.stringify(extrinsics.map(e => e && [e.id, e.x ?? e.dx, e.y ?? e.dy, e.z, e.roll, e.pitch, e.yaw ?? e.thetaDeg]));
    if (zoneSig !== this.zoneExtrinsicSignature) {
      this.zoneExtrinsicSignature = zoneSig;
      this.rebuildZones(extrinsics);
    }

    const state = this.fusion.select(names.length, performance.now(), paused);
    let total = 0, online = 0;

    for (const id of [...this.layers.keys()]) {
      if (id >= names.length) this.removeLayer(id);
    }

    // Auto-scale Grid và Axes
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const id in extrinsics) {
      const e = extrinsics[id];
      if (e.x < minX) minX = e.x; if (e.x > maxX) maxX = e.x;
      if (e.y < minY) minY = e.y; if (e.y > maxY) maxY = e.y;
      if (e.z < minZ) minZ = e.z; if (e.z > maxZ) maxZ = e.z;
    }

    if (minX === Infinity) { minX = -10; maxX = 10; minY = -10; maxY = 10; minZ = 0; maxZ = 10; }
    const maxSpan = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 20) * 1.5;

    this.grid.scale.setScalar(maxSpan / this.baseGridSize);
    this.axes.scale.setScalar(maxSpan / this.baseAxesSize);
    const labelOffset = maxSpan + 1.5;
    this.axisLabels['X'].position.set(labelOffset, 0, 0);
    this.axisLabels['Y'].position.set(0, labelOffset, 0);
    this.axisLabels['Z'].position.set(0, 0, labelOffset);

    for (const entry of state.layers) {
      const { id, frame: f, stale, delta } = entry;
      const name = names[id] || 'LiDAR ' + (id + 1);
      const l = this.layers.get(id) || this.makeLayer(id, name);

      if (l.name !== name) {
        l.group.remove(l.label); this.disposeObject(l.label);
        l.label = this.label(name, this.palette[id % this.palette.length]);
        l.group.add(l.label); l.name = name;
      }

      const e = extrinsics[id] || {};
      const ox = f?.ox ?? e.x ?? e.dx ?? 0;
      const oy = f?.oy ?? e.y ?? e.dy ?? 0;
      const oz = f?.oz ?? e.z ?? 0;

      l.sensor.position.set(ox, oy, oz);
      l.sensor.rotation.set(
        f?.roll ?? (e.roll || 0) * Math.PI / 180,
        f?.pitch ?? (e.pitch || 0) * Math.PI / 180,
        f?.oth ?? (e.yaw ?? e.thetaDeg ?? 0) * Math.PI / 180,
        'ZYX'
      );
      l.label.position.set(ox, oy, oz + 0.5);
      l.group.visible = l.check.checked;
      if (l.scanZone) l.scanZone.visible = this.showPlanes && l.check.checked;
      if (l.blindZone) l.blindZone.visible = this.showPlanes && l.check.checked;
      
      l.points.visible = true; 
      l.points.material.size = this.pointSize;
      
      if (stale) {
        l.points.material.opacity = 0.3;
      } else {
        l.points.material.opacity = 1.0;
        online++;
      }
      
      if (l.check.checked && f) total += f.n;
      
      if (f && f.xyz && (l.frame !== f || l.decimation !== this.decimation)) {
        const n = Math.ceil(f.n / this.decimation);
        let attr = l.points.geometry.getAttribute('position');
        if (attr.count < n) {
          l.points.geometry.dispose();
          l.points.geometry = new THREE.BufferGeometry();
          attr = new THREE.BufferAttribute(new Float32Array(Math.max(n, attr.count * 2) * 3), 3);
          attr.setUsage(THREE.DynamicDrawUsage);
          l.points.geometry.setAttribute('position', attr);
        }
        for (let i = 0, k = 0; i < f.n; i += this.decimation, k += 3) {
          attr.array[k] = f.xyz[i * 3]; 
          attr.array[k + 1] = f.xyz[i * 3 + 1]; 
          attr.array[k + 2] = f.xyz[i * 3 + 2];
        }
        attr.needsUpdate = true;
        l.points.geometry.setDrawRange(0, n);
        l.frame = f; 
        l.decimation = this.decimation;
      }
      
      l.text.textContent = ` ${name}: Z=${oz.toFixed(2)} m | ${f?.n || 0} pts | ${stale ? 'STALE' : (delta >= 0 ? '+' : '') + delta.toFixed(1) + ' ms'}`;
      l.text.style.color = stale ? '#a33' : this.palette[id % this.palette.length];
    }

    document.getElementById('fusionSummary').textContent =
      `LiDAR online: ${online}/${names.length} | Total points: ${total}\n` +
      `Fusion timestamp: ${state.timestamp === null ? '—' : state.timestamp.toFixed(3) + ' ms'}\n` +
      `Max timestamp delta: ${state.maxDelta.toFixed(1)} ms\n` +
      `Grid: Auto-scaled • Z up • Drag: rotate • Right drag: pan • Wheel: zoom`;

    this.updateTargets(ev);
    this.updateDistanceInfoAll();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

    dispose() {
    this.observer.disconnect();
    this.controls.dispose();
    this.clearZones();
    for (const entry of this.targetMeshes.values()) {
      this.disposeObject(entry.group);
      this.disposeObject(entry.label);
    }
    this.targetMeshes.clear();
    this.alarmBanner.remove();
    this.clear();
    this.disposeObject(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

window.Lidar3D = Lidar3D;
window.dispatchEvent(new Event('lidar3d-ready'));