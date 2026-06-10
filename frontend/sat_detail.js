window.SatDetailView = (function() {
  var API_BASE = 'http://localhost:8080';
  var selectedSatelliteId = null;

  function showToast(msg, type) {
    var c = document.getElementById('toast-container');
    var t = document.createElement('div');
    t.className = 'toast ' + (type || 'info');
    t.textContent = msg;
    c.appendChild(t);
    requestAnimationFrame(function() { t.classList.add('show'); });
    setTimeout(function() {
      t.classList.remove('show');
      setTimeout(function() { c.removeChild(t); }, 300);
    }, 3000);
  }

  function riskColorCSS(level) {
    if (level === 'danger') return '#ff4444';
    if (level === 'warning') return '#ffd700';
    return '#4ecdc4';
  }

  function initUI() {
    document.getElementById('detail-close').addEventListener('click', closeDetailPanel);

    var toggleBtn = document.getElementById('alert-panel-toggle');
    var alertPanel = document.getElementById('alert-panel');
    var panelCollapsed = false;
    toggleBtn.addEventListener('click', function() {
      panelCollapsed = !panelCollapsed;
      alertPanel.classList.toggle('collapsed', panelCollapsed);
      toggleBtn.innerHTML = panelCollapsed ? '&#x25C0;' : '&#x25B6;';
    });
  }

  function openDetailPanel(satId) {
    selectedSatelliteId = satId;
    var sat = window.Sat3DViewer.getSatData(satId);
    if (!sat) return;

    document.getElementById('detail-title').textContent = sat.name + ' (SAT-' + satId + ')';
    var content = document.getElementById('detail-content');
    content.innerHTML = '';

    var risk = sat.collision_risk_level;
    content.innerHTML +=
      '<div class="detail-section">' +
        '<h4>碰撞风险</h4>' +
        '<span class="risk-badge ' + risk + '">' + risk.toUpperCase() + '</span>' +
      '</div>';

    var oe = sat.orbital_elements;
    content.innerHTML +=
      '<div class="detail-section">' +
        '<h4>轨道根数</h4>' +
        '<table class="detail-table">' +
          '<tr><td>半长轴 (a)</td><td>' + oe.semi_major_axis.toFixed(2) + ' km</td></tr>' +
          '<tr><td>离心率 (e)</td><td>' + oe.eccentricity.toFixed(6) + '</td></tr>' +
          '<tr><td>轨道倾角 (i)</td><td>' + (oe.inclination * 180 / Math.PI).toFixed(4) + '\u00B0</td></tr>' +
          '<tr><td>升交点赤经 (\u03A9)</td><td>' + (oe.raan * 180 / Math.PI).toFixed(4) + '\u00B0</td></tr>' +
          '<tr><td>近地点幅角 (\u03C9)</td><td>' + (oe.arg_perigee * 180 / Math.PI).toFixed(4) + '\u00B0</td></tr>' +
          '<tr><td>真近点角 (\u03BD)</td><td>' + (oe.true_anomaly * 180 / Math.PI).toFixed(4) + '\u00B0</td></tr>' +
        '</table>' +
      '</div>';

    var pos = sat.current_position;
    var vel = sat.velocity;
    content.innerHTML +=
      '<div class="detail-section">' +
        '<h4>位置 / 速度 (ECI)</h4>' +
        '<table class="detail-table">' +
          '<tr><td>位置 X</td><td>' + pos.x.toFixed(2) + ' km</td></tr>' +
          '<tr><td>位置 Y</td><td>' + pos.y.toFixed(2) + ' km</td></tr>' +
          '<tr><td>位置 Z</td><td>' + pos.z.toFixed(2) + ' km</td></tr>' +
          '<tr><td>速度 X</td><td>' + vel.x.toFixed(3) + ' km/s</td></tr>' +
          '<tr><td>速度 Y</td><td>' + vel.y.toFixed(3) + ' km/s</td></tr>' +
          '<tr><td>速度 Z</td><td>' + vel.z.toFixed(3) + ' km/s</td></tr>' +
        '</table>' +
      '</div>';

    var prop = sat.propellant;
    var propPct = Math.max(0, Math.min(100, prop.remaining));
    var propColor = propPct > 50 ? '#4ecdc4' : propPct > 20 ? '#ffd700' : '#ff4444';
    content.innerHTML +=
      '<div class="detail-section">' +
        '<h4>推进剂</h4>' +
        '<table class="detail-table">' +
          '<tr><td>剩余</td><td>' + prop.remaining.toFixed(1) + '%</td></tr>' +
          '<tr><td>消耗速率</td><td>' + prop.consumption_rate.toFixed(4) + ' %/h</td></tr>' +
          '<tr><td>预计寿命</td><td>' + prop.estimated_lifetime_hours.toFixed(0) + ' h</td></tr>' +
        '</table>' +
        '<div class="propellant-bar-bg"><div class="propellant-bar-fill" style="width:' + propPct + '%;background:' + propColor + '"></div></div>' +
      '</div>';

    var chartDiv = document.createElement('div');
    chartDiv.className = 'detail-section';
    chartDiv.innerHTML = '<h4>推进剂消耗趋势 (24h)</h4><canvas id="propellant-chart"></canvas>';
    content.appendChild(chartDiv);
    loadPropellantChart(satId);

    var telemDiv = document.createElement('div');
    telemDiv.className = 'detail-section';
    telemDiv.innerHTML = '<h4>遥测历史 (1h)</h4><div id="telemetry-table-container">加载中...</div>';
    content.appendChild(telemDiv);
    loadTelemetryTable(satId);

    document.getElementById('detail-panel').classList.add('open');
  }

  function closeDetailPanel() {
    document.getElementById('detail-panel').classList.remove('open');
    selectedSatelliteId = null;
  }

  function loadPropellantChart(satId) {
    fetch(API_BASE + '/api/satellites/' + satId + '/propellant?hours=24')
      .then(function(r) { return r.json(); })
      .then(function(data) {
        drawPropellantChart(data);
      })
      .catch(function() {
        var canvas = document.getElementById('propellant-chart');
        if (canvas) {
          var ctx = canvas.getContext('2d');
          ctx.fillStyle = '#6888b8';
          ctx.font = '12px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText('数据加载失败', canvas.width / 2, canvas.height / 2);
        }
      });
  }

  function drawPropellantChart(data) {
    var canvas = document.getElementById('propellant-chart');
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    var ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    var w = rect.width;
    var h = rect.height;

    ctx.fillStyle = 'rgba(5,5,30,0.6)';
    ctx.fillRect(0, 0, w, h);

    if (!data || data.length === 0) {
      ctx.fillStyle = '#6888b8';
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('暂无数据', w / 2, h / 2);
      return;
    }

    var pad = { top: 10, right: 10, bottom: 20, left: 35 };
    var plotW = w - pad.left - pad.right;
    var plotH = h - pad.top - pad.bottom;

    var propValues = data.map(function(d) { return d.propellant_remaining; });
    var minVal = Math.min.apply(null, propValues);
    var maxVal = Math.max.apply(null, propValues);
    var range = maxVal - minVal;
    if (range < 0.1) { minVal -= 1; maxVal += 1; range = maxVal - minVal; }

    ctx.strokeStyle = 'rgba(100,140,255,0.1)';
    ctx.lineWidth = 0.5;
    for (var gi = 0; gi <= 4; gi++) {
      var gy = pad.top + (gi / 4) * plotH;
      ctx.beginPath();
      ctx.moveTo(pad.left, gy);
      ctx.lineTo(pad.left + plotW, gy);
      ctx.stroke();
      ctx.fillStyle = '#6888b8';
      ctx.font = '9px sans-serif';
      ctx.textAlign = 'right';
      var gVal = maxVal - (gi / 4) * range;
      ctx.fillText(gVal.toFixed(1) + '%', pad.left - 4, gy + 3);
    }

    ctx.beginPath();
    ctx.strokeStyle = '#4ecdc4';
    ctx.lineWidth = 1.5;
    data.forEach(function(d, i) {
      var x = pad.left + (i / (data.length - 1)) * plotW;
      var y = pad.top + (1 - (d.propellant_remaining - minVal) / range) * plotH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    var grad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    grad.addColorStop(0, 'rgba(78,205,196,0.15)');
    grad.addColorStop(1, 'rgba(78,205,196,0.0)');
    ctx.lineTo(pad.left + plotW, pad.top + plotH);
    ctx.lineTo(pad.left, pad.top + plotH);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    if (data.length > 0) {
      var lastTime = new Date(data[data.length - 1].timestamp);
      var firstTime = new Date(data[0].timestamp);
      ctx.fillStyle = '#6888b8';
      ctx.font = '9px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(firstTime.toLocaleTimeString(), pad.left, h - 4);
      ctx.fillText(lastTime.toLocaleTimeString(), pad.left + plotW, h - 4);
    }
  }

  function loadTelemetryTable(satId) {
    fetch(API_BASE + '/api/satellites/' + satId + '/telemetry?hours=1')
      .then(function(r) { return r.json(); })
      .then(function(data) {
        renderTelemetryTable(data);
      })
      .catch(function() {
        var c = document.getElementById('telemetry-table-container');
        if (c) c.innerHTML = '<div style="color:#6888b8;font-size:11px;padding:8px">数据加载失败</div>';
      });
  }

  function renderTelemetryTable(data) {
    var c = document.getElementById('telemetry-table-container');
    if (!c) return;
    if (!data || data.length === 0) {
      c.innerHTML = '<div style="color:#6888b8;font-size:11px;padding:8px">暂无遥测数据</div>';
      return;
    }

    var html = '<table class="telemetry-table">' +
      '<thead><tr><th>时间</th><th>SMA (km)</th><th>离心率</th><th>推进剂</th></tr></thead><tbody>';

    data.slice().reverse().forEach(function(d) {
      var ts = d.timestamp ? new Date(d.timestamp).toLocaleTimeString() : '--';
      var sma = d.semi_major_axis != null ? d.semi_major_axis.toFixed(2) : '--';
      var ecc = d.eccentricity != null ? d.eccentricity.toFixed(6) : '--';
      var prop = d.propellant_remaining != null ? d.propellant_remaining.toFixed(1) + '%' : '--';
      html += '<tr><td>' + ts + '</td><td>' + sma + '</td><td>' + ecc + '</td><td>' + prop + '</td></tr>';
    });

    html += '</tbody></table>';
    c.innerHTML = html;
  }

  function loadAlerts() {
    fetch(API_BASE + '/api/alerts')
      .then(function(r) { return r.json(); })
      .then(function(alerts) {
        renderAlertPanel(alerts);
      })
      .catch(function(err) {
        console.error('Failed to load alerts:', err);
      });
  }

  function renderAlertPanel(alerts) {
    var list = document.getElementById('alert-list');
    list.innerHTML = '';
    var activeAlerts = alerts.filter(function(a) { return a.alert_level > 0; });
    document.getElementById('alert-count-badge').textContent = activeAlerts.length;
    document.getElementById('stat-alerts').textContent = activeAlerts.length;

    if (activeAlerts.length === 0) {
      list.innerHTML = '<div style="padding:20px;text-align:center;color:#6888b8;font-size:12px">暂无活跃碰撞预警</div>';
      return;
    }

    activeAlerts.sort(function(a, b) { return b.alert_level - a.alert_level; });

    activeAlerts.forEach(function(alert) {
      var card = document.createElement('div');
      card.className = 'alert-card level-' + alert.alert_level;

      var probStr = alert.collision_probability < 0.001
        ? alert.collision_probability.toExponential(2)
        : alert.collision_probability.toFixed(4);

      var badgeClass = alert.alert_level === 2 ? 'level-2' : 'level-1';
      var badgeText = alert.alert_level === 2 ? 'DANGER' : 'WARNING';

      var tcaStr = alert.tca ? new Date(alert.tca).toLocaleString() : '--';

      card.innerHTML =
        '<div class="alert-pair">' +
          alert.satellite_name_1 + ' \u2194 ' + alert.satellite_name_2 +
          '<span class="alert-badge ' + badgeClass + '">' + badgeText + '</span>' +
        '</div>' +
        '<div class="alert-detail">' +
          'TCA: <span>' + tcaStr + '</span><br>' +
          '最小距离: <span>' + alert.miss_distance.toFixed(1) + ' km</span><br>' +
          '碰撞概率: <span>' + probStr + '</span>' +
        '</div>';

      if (alert.alert_level === 2) {
        var btn = document.createElement('button');
        btn.className = 'btn-avoidance';
        btn.textContent = alert.maneuver_planned ? '规避机动已规划' : '计算规避机动';
        btn.disabled = alert.maneuver_planned;
        btn.addEventListener('click', function() {
          computeAvoidance(alert.alert_id, btn);
        });
        card.appendChild(btn);
      }

      list.appendChild(card);
    });
  }

  function computeAvoidance(alertId, btn) {
    btn.disabled = true;
    btn.textContent = '计算中...';
    fetch(API_BASE + '/api/compute-avoidance/' + alertId, { method: 'POST' })
      .then(function(r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      })
      .then(function(data) {
        btn.textContent = '规避机动已规划';
        showToast('规避机动计算完成', 'success');
        loadAlerts();
      })
      .catch(function(err) {
        btn.disabled = false;
        btn.textContent = '计算规避机动';
        showToast('规避机动计算失败: ' + err.message, 'error');
      });
  }

  function loadOverview() {
    fetch(API_BASE + '/api/constellation/overview')
      .then(function(r) { return r.json(); })
      .then(function(data) {
        document.getElementById('stat-total').textContent = data.total_satellites;
        document.getElementById('stat-alerts').textContent = data.active_alerts;
        document.getElementById('stat-propellant').textContent = data.avg_propellant.toFixed(1) + '%';
        document.getElementById('stat-coverage').textContent = data.coverage_status;
      })
      .catch(function(err) {
        console.error('Failed to load overview:', err);
      });
  }

  return {
    init: initUI,
    openDetailPanel: openDetailPanel,
    closeDetailPanel: closeDetailPanel,
    loadAlerts: loadAlerts,
    loadOverview: loadOverview,
    startAutoRefresh: function() {
      setInterval(function() {
        loadOverview();
        loadAlerts();
      }, 30000);
    }
  };
})();
