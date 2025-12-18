/*******************************************************************************
 * Copyright (c) 2025  Carnegie Mellon University and Miraikan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

const urlParams = new URLSearchParams(window.location.search);
let directory_data = {};
let node_names = {};
let tour_names = {};
let last_data = {};
let __debug__ = {};
let current_lang = '';
let current_handleside = '';
let cuurrent_touchmode = '';
let current_voicerate = '';
let cuurrent_chatvisible = '';
let current_userapp_level = '';
let current_system_level = '';
let add_destination_dialog;
let generic_confirm_dialog;
let diagnostics_level = -1;
let last_stop_time;

function post_data(url, body) {
    fetch(url, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(body)
    })
        .then(response => response.json())
        .catch(error => console.error('Error:', error));
}

function share(data) {
    const defaults = {
        value: '', flag1: false, flag2: false, location: 0, length: 0, info_id: new Date().getTime() * 1000000
    };
    post_data('/publish/', { event: 'share', data: JSON.stringify({ ...defaults, ...data }) });
}

function speak(data) {
    const defaults = {
        rate: 50, pitch: 50, volume: 50, lang: current_lang, voice: 'male', force: false, priority: 90, timeout: 2.0, channels: 3, request_id: new Date().getTime() * 1000000
    };
    post_data('/publish/', { event: 'speak', data: JSON.stringify({ ...defaults, ...data }) });
}

function manage(data) {
    post_data('/manage/', { data: data });
}

function speak_text() {
    speak({ text: document.getElementById('speak_text').value });
}

function toggleBox(legend) {
    const fs = legend.closest("fieldset");
    fs.classList.toggle("collapsed");
    document.querySelectorAll("fieldset.toggle").forEach(el => {
        if (el !== fs) {
            el.classList.add("collapsed");
        }
    });
}

function add_destination(node) {
    const tour = last_data['share.Tour']?.at(-1);
    if (tour) {
        let count = tour.currentDestination ? 1 : 0 + (tour.destinations ?? []).length;
        if (count > 0) {
            document.getElementById('destination_count').textContent = count;
            add_destination_dialog.dataset.node = node;
            add_destination_dialog.showModal();
            return;
        }
    }
    share({ type: 'OverrideDestination', value: node });
}

function add_destination_close(event) {
    const dialog = event.target;
    let clear = false, first = false;
    switch (dialog.returnValue) {
        case "replace":
            clear = true;
            break;
        case "first":
            first = true;
            break;
        case "cancel":
            return;
    }
    share({ type: 'OverrideDestination', value: dialog.dataset.node, flag1: clear, flag2: first });
}

function generic_confirm(prompt_html, yes_text, no_text, type, value) {
    document.getElementById('generic_prompt').innerHTML = prompt_html;
    document.getElementById('generic_yes').textContent = yes_text;
    document.getElementById('generic_no').textContent = no_text;
    generic_confirm_dialog.generic_type = type;
    generic_confirm_dialog.generic_value = value;
    generic_confirm_dialog.showModal();
}

function generic_confirm_close(event) {
    const dialog = event.target;
    if (dialog.returnValue == 'yes') {
        switch (dialog.generic_type) {
            case 'manage':
                manage(dialog.generic_value);
                break;
            case 'share':
                share(dialog.generic_value);
                break;
        }
    }
}


function set_tour(tour) {
    generic_confirm(`ツアーを送信：${tour_name(tour)}<br>ユーザーのツアーが上書きされます`, 'ツアーを送信', 'キャンセル', 'share', { type: 'OverrideTour', value: tour });
}

function clear_destinations(count) {
    generic_confirm(`${count}個の目的地が設定されています。<br>全てキャンセルしてよろしいですか？`, 'すべての目的地を中止', 'いいえ', 'share', { type: 'ClearDestinations' });
}

function skip(node) {
    generic_confirm('本当にこの目的地をスキップしたいですか？', `${destination_name(node)}をスキップ`, 'いいえ', 'share', { type: 'Skip', value: node });
}

function renderSections(sections, level = 0) {
    let html = "";
    for (const section of sections) {
        if (section.items) {
            if (level == 0) {
                html += `<fieldset><legend>${section.title}</legend>`;
            }
            for (const item of section.items) {
                if (item.content?.sections) {
                    html += `<fieldset class="collapsed toggle"><legend onclick="toggleBox(this)">${item.title}</legend>`;
                    html += renderSections(item.content.sections, level + 1);
                    html += `</fieldset>`;
                } else {
                    if (item.nodeID) {
                        let nodeID = item.nodeID;
                        for (const dest of directory_data.destinations) {
                            if (dest.value == nodeID) {
                                if ('arrivalAngle' in dest) {
                                    nodeID = `${nodeID}@${dest.arrivalAngle}`
                                }
                                break;
                            }
                        }
                        html += `<div onclick="add_destination('${nodeID}')">${item.title ?? nodeID}</div>`;
                    }
                }
            }
            if (level == 0) {
                html += `</fieldset>`;
            }
        }
    }
    return html;
}

function renderTours(tours) {
    let html = "";
    for (const tour of tours) {
        html += `<div onclick="set_tour('${tour.tour_id}')">${tour_names[tour.tour_id]}${tour.debug ? ' (Debug)' : ''}</div>`;
    }
    return html;
}

function destination_name(node) {
    return node_names[node.split(/[#@]/)[0]] ?? node;
}

function tour_name(tour) {
    return tour_names[tour] ?? tour;
}

function get_current_tourname(data) {
    const tour = data['share.Tour']?.at(-1);
    if (tour?.id && (tour.currentDestination || (tour.destinations?.length > 0))) {
        return tour_names[tour.id] || "カスタムツアー";
    }
    return '';
}

function renderCurrentDestinations(data) {
    let html = "";
    const tour = data['share.Tour']?.at(-1);
    let skip = true;
    if (tour) {
        const tour_name = get_current_tourname(data);
        if (tour_name) {
            html += `<div class="tour-name">ツアー：${tour_name}</div>`;
        }
        if (tour.currentDestination) {
            const name = destination_name(tour.currentDestination);
            html += `<div class="current-destination">${name}に移動中 <button onclick="skip('${tour.currentDestination}')">スキップ</button></div>`;
            skip = false;
        }
        for (const destination of tour.destinations ?? []) {
            const name = destination_name(destination);
            if (skip) {
                html += `<div class="next-destination">${name} <button onclick="skip('${destination}')">スキップ</button>`;
                skip = false;
            } else {
                html += `<div>${name}`;
            }
            html += '</div>';
        }
        let count = tour.currentDestination ? 1 : 0 + (tour.destinations ?? []).length;
        if (count > 0) {
            html += `<button onclick="clear_destinations(${count})" style="margin-top:1em">ナビゲーションを中止</button>`;
        }
    }
    return html;
}

function renderChatHistories(data) {
    let html = '';
    for (const item of data['share.ChatStatus'] ?? []) {
        html += `<div class="bubble ${item.user.toLowerCase()}">${item.text}</div>`;
    }
    return html;
}

function renderSpeakHistories(data) {
    let html = '';
    for (const item of data['share.Speak'] ?? []) {
        html += renderHistoryItem(item, 'speak')
    }
    return html;
}

function renderButtonHistories(data) {
    let html = "";
    for (const item of data.button ?? []) {
        html += renderHistoryItem(item, 'button');
    }
    return html;
}

function renderDestinationHistories(data) {
    let html = "";
    for (const item of data.destination ?? []) {
        html += renderHistoryItem(item, 'destination');
    }
    return html;
}

function renderHistories(data) {
    const a = (data['share.Speak'] ?? []).map(item => ({ ...item, _type: 'speak' }));
    const b = (data.button ?? []).map(item => ({ ...item, _type: 'button' }));
    const c = (data.destination ?? []).map(item => ({ ...item, _type: 'destination' }));
    const merged = [...a, ...b, ...c].sort((x, y) => x.timestamp.localeCompare(y.timestamp)).slice(-15);
    let html = '';
    for (const item of merged) {
        html += renderHistoryItem(item, item._type);
    }
    return html;
}

function renderHistoryItem(item, type) {
    const timestamp = new Date(item.timestamp).toLocaleTimeString();
    const data = item.data;
    let text;
    switch (type) {
        case 'speak':
            text = data;
            break
        case 'button':
            const button_names = ['上', '下', '左', '右', '中央'];
            if (data.type == 'click') {
                text = `${button_names[data.buttons - 1]}ボタンを`;
                text += data.count == 1 ? 'クリック' : data.count == 2 ? 'ダブルクリック' : 'トリプルクリック';
            } else if (data.type == 'holddown') {
                text = `${button_names[data.holddown - 1]}ボタンを長押し ${data.duration}秒`;
            }
            break;
        case 'destination':
            text = data == '__arrived__' ? '到着' : data == '__cancel__' ? 'キャンセル' : `${destination_name(data)}に移動`;
            break;
    }
    return `<div class="history-${type}">${new Date(item.timestamp).toLocaleTimeString()} : ${text}</div>`;
}

function renderSystemStatus(data) {
    let html = "";
    const status = data.system_status?.at(-1) ?? {};
    const sorted = [...(status.diagnostics ?? [])].sort((a, b) =>
        a.name.localeCompare(b.name)
    );
    for (const diag of sorted) {
        diagnostics_level = Math.max(diagnostics_level, diag.level);
        if (diag.name == '/CaBot' || diag.level > 1) {
            html += '<table>';
            html += `<thead><tr><td>${diag.name}</td><td>${diag.message}</td></tr></thead>`;
            html += '<tbody>';
            for (const value of diag.values ?? []) {
                html += `<tr><td>${value.key}</td><td>${value.value}</td></tr>`;
            }
            html += '</tbody>';
            html += `</table>`;
        }
    }
    return html;
}

function renderTemperature(data) {
    let html = '<table><tbody>';
    const sorted = [...(data.device_status?.at(-1)?.devices ?? [])].sort((a, b) =>
        a.model.localeCompare(b.model)
    );
    for (const device of sorted) {
        if (device.type == 'Suitcase Temperature') {
            const temperature = parseFloat(device.message) || 0;
            const percent = Math.min(Math.max(temperature - 10, 0), 50) / 50 * 100;
            const cls = device.level?.toLowerCase() || '';
            const level = ['ok', 'warning', 'error'].indexOf(cls);
            diagnostics_level = Math.max(diagnostics_level, level);
            html += `
                <tr>
                    <td>${device.model.replaceAll('_', ' ')}</td>
                    <td><div class="bar-container">
                        <div class="bar-fill ${cls}" style="width:${percent}%"></div>
                        <span class="bar-label">${device.message}</span>
                    </div></td>
                </tr>
            `;
        }
    }
    html += '</tbody></table>';
    return html;
}

function renderBattery(data) {
    let html = '<table><tbody>';
    for (const battery of data.battery_status?.at(-1)?.values ?? []) {
        const percent = parseFloat(battery.value) || 0;
        const level = percent <= 10 ? 2 : percent <= 20 ? 1 : 0;
        diagnostics_level = Math.max(diagnostics_level, level);
        const cls = ['', 'warning', 'error'][level];
        html += `
            <tr>
                <td>${battery.key}</td>
                <td><div class="bar-container">
                    <div class="bar-fill ${cls}" style="width:${percent}%"></div>
                    <span class="bar-label">${battery.value}</span>
                </div></td>
            </tr>
        `;
    }
    html += '</tbody></table>';
    return html;
}

function renderTouchLevel(data) {
    const level = data.average_touch?.at(-1) ?? -1
    let percent = Math.max(level * 100, 0);
    let text = 'オン';
    let cls = '';
    if (level < 0) {
        text = '未検出';
    } else if (level == 0) {
        text = 'オフ';
    } else if (level < 0.25) {
        cls = "error";
        text = `${Math.floor(level * 100)}%`;
    }
    return `
        <div class="bar-container">
            <div class="bar-fill ${cls}" style="width:${percent}%"></div>
            <span class="bar-label">${text}</span>
        </div>
    `;
}

function renderSpeedLevel(data) {
    const level = data.average_speed?.at(-1) ?? -1
    let percent = Math.max(level * 100, 0);
    let text = level < 0 ? '未検出' : level == 0 ? '停止中' : `${level.toFixed(3)}`;
    let cls = '';
    if (!last_stop_time || level != 0) {
        last_stop_time = new Date();
    } else if (level == 0) {
        const elapsed = Math.floor((new Date() - last_stop_time) / 1000);
        text += `（${elapsed}秒）`;
        const touch_level = data.average_touch?.at(-1) ?? -1;
        if (elapsed >= 10 && touch_level > 0) {
            cls = elapsed >= 30 ? 'error' : 'warning';
            percent = 100;
        }
    }
    return `
        <div class="bar-container">
            <div class="bar-fill ${cls}" style="width:${percent}%"></div>
            <span class="bar-label">${text}</span>
        </div>
    `;
}

function renderImuData(data, type, warning_level = 10, error_level = 30) {
    const level = (data.imu_data?.at(-1) ?? {})[type] ?? -1;
    let percent = Math.max(level * 100 / 90, 0);
    let text = level < 0 ? '未検出' : `${level.toFixed(1)}°`;
    let cls = level < warning_level ? '' : level < error_level ? 'warning' : 'error';
    return `
        <div class="bar-container">
            <div class="bar-fill ${cls}" style="width:${percent}%"></div>
            <span class="bar-label">${text}</span>
        </div>
    `;
}

function renderDiagnosticsLevel() {
    let percent = 0;
    let cls = '';
    let text = 'なし';
    if (diagnostics_level >= 0) {
        percent = 100;
        text = '正常';
        if (diagnostics_level >= 1) {
            text = '要確認';
            cls = diagnostics_level >= 2 ? 'error' : 'warning';
        }
    }
    return `
        <div class="bar-container">
            <div class="bar-fill ${cls}" style="width:${percent}%"></div>
            <span class="bar-label">${text}</span>
        </div>
    `;
}

function build_index() {
    node_names = {};
    for (const feature of directory_data.features) {
        const p = feature.properties ?? {};
        if (p.facil_id) {
            const name = p[`name_${current_lang}`];
            if (name) {
                for (let i = 1; i <= 9; i++) {
                    const node = p[`ent${i}_node`];
                    if (node) {
                        const ent_name = p[`ent${i}_n`];
                        node_names[node] = `${name}${ent_name ? ' ' + ent_name : ''}`;
                    }
                }
            }
        }
    }

    tour_names = {}
    for (const tour of directory_data.tours) {
        tour_names[tour.tour_id] = tour['title-' + current_lang];
    }
}

function replaceHTML(id, html) {
    const element = document.getElementById(id);
    if (element && element.innerHTML != html) {
        element.innerHTML = html;
    }
}

function replaceText(id, text) {
    const element = document.getElementById(id);
    if (element && element.textContent != text) {
        element.textContent = text;
    }
}

function set_highlight(language, voicerate, handleside, touchmode, chatvisible) {
    if (language != undefined) {
        document.querySelectorAll('[data-language]').forEach(el => {
            if (el.dataset.language === language) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (handleside != undefined) {
        document.querySelectorAll('[data-handleside]').forEach(el => {
            if (el.dataset.handleside === handleside) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (touchmode != undefined) {
        document.querySelectorAll('[data-touchmode]').forEach(el => {
            if (el.dataset.touchmode === touchmode) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (chatvisible != undefined) {
        document.querySelectorAll('[data-chatvisible]').forEach(el => {
            if (el.dataset.chatvisible === chatvisible) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (voicerate != undefined) {
        if (voicerate < (0.2 + 0.35) / 2) {
            voicerate = 'very-slow';
        } else if (voicerate < (0.35 + 0.5) / 2) {
            voicerate = 'slow';
        } else if (voicerate < (0.5 + 0.6) / 2) {
            voicerate = 'normal';
        } else if (voicerate < (0.6 + 0.7) / 2) {
            voicerate = 'fast';
        } else if (voicerate < (0.7 + 0.8) / 2) {
            voicerate = 'very-fast';
        } else {
            voicerate = 'super-fast';
        }
        document.querySelectorAll('[data-voicerate]').forEach(el => {
            if (el.dataset.voicerate === voicerate) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
}

function set_disabled(userapp_level, system_level) {
    if (userapp_level != undefined) {
        document.querySelectorAll('[data-userapp]').forEach(el => {
            const disabled = el.dataset.userapp != userapp_level;
            if (disabled) {
                el.classList.add('disabled');
            } else {
                el.classList.remove('disabled');
            }
            el.querySelectorAll('button, input, select, textarea').forEach(e => {
                e.disabled = disabled;
            });
        });
        document.getElementById('user_app_status').innerHTML = `
            <div class="bar-container">
                <div class="bar-fill ${userapp_level == 'OK' ? '' : 'error'}" style="width:100%"></div>
                <span class="bar-label">${userapp_level == 'OK' ? '接続中' : '接続されていません'}</span>
            </div>
        `;
    }
    if (system_level != undefined) {
        document.querySelectorAll('[data-system]').forEach(el => {
            const disabled = el.dataset.system != system_level;
            if (disabled) {
                el.classList.add('disabled');
            } else {
                el.classList.remove('disabled');
            }
            if (el.matches('button, input, select, textarea')) {
                el.disabled = disabled;
            }
            el.querySelectorAll('button, input, select, textarea').forEach(e => {
                e.disabled = disabled;
            });
        });
    }
}

function handle_last_data() {
    fetch('/last_data/', {})
        .then(response => response.json())
        .then(data => {
            last_data = data;
            let lang = data['share.ChangeLanguage']?.at(-1);
            if (lang == 'zh-Hans') {
                lang = 'zh-CN';
            }
            let language_changed, voicerate_changed, handleside_changed, touchmode_changed, chatvisible_changed;
            if (lang && lang != current_lang) {
                console.log(`Switch to ${lang}`);
                current_lang = lang;
                language_changed = lang
                build_index();
                replaceHTML('destinations', renderSections(directory_data.sections[current_lang]));
                replaceHTML('tours', renderTours(directory_data.tours));
            }
            diagnostics_level = -1;
            replaceHTML('histories', renderHistories(data));
            replaceHTML('chat_histories', renderChatHistories(data));
            replaceHTML('navigation_histories', renderDestinationHistories(data));
            replaceHTML('current_destinations', renderCurrentDestinations(data));
            replaceHTML('system_info', renderSystemStatus(data));
            replaceHTML('touch_level', renderTouchLevel(data));
            replaceHTML('speed_level', renderSpeedLevel(data));
            replaceHTML('battery', renderBattery(data));
            replaceHTML('temperature', renderTemperature(data));
            replaceText('cabot_name', data.cabot_name?.at(-1) ?? '未接続');
            replaceHTML('diagnostics_level', renderDiagnosticsLevel());
            replaceHTML('pitch_level', renderImuData(data, 'pitch'));
            replaceHTML('roll_level', renderImuData(data, 'roll'));
            if (document.getElementById('debug-info').style.display != 'none' && !document.getElementById('pause_debug_update').checked) {
                document.getElementById('messages').innerText = JSON.stringify(data, null, 2);
            }
            const voicerate = data['share.ChangeUserVoiceRate']?.at(-1);
            if (voicerate && voicerate != current_voicerate) {
                current_voicerate = voicerate;
                voicerate_changed = voicerate;
            }
            const handleside = data['share.ChangeHandleSide']?.at(-1);
            if (handleside && handleside != current_handleside) {
                current_handleside = handleside;
                handleside_changed = handleside;
            }
            const touchmode = data['share.ChangeTouchMode']?.at(-1);
            if (touchmode && touchmode != cuurrent_touchmode) {
                cuurrent_touchmode = touchmode;
                touchmode_changed = touchmode;
            }
            const chatvisible = String(data['share.ChatStatus.visible']?.at(-1) ?? false);
            if (chatvisible != cuurrent_chatvisible) {
                cuurrent_chatvisible = chatvisible;
                chatvisible_changed = chatvisible;
            }
            set_highlight(language_changed, voicerate_changed, handleside_changed, touchmode_changed, chatvisible_changed);

            let userapp_level_changed, system_level_changed;
            let userapp_level = '';
            for (const device of data.device_status?.at(-1)?.devices ?? []) {
                if (device.type == 'User App') {
                    userapp_level = device.level;
                    break;
                }
            }
            if (userapp_level != current_userapp_level) {
                current_userapp_level = userapp_level;
                userapp_level_changed = userapp_level;
            }
            const system_status = data.system_status?.at(-1) ?? {};
            const system_level = system_status.level == 'Inactive' && system_status.diagnostics?.length > 0 ? 'Active' : system_status.level;
            if (system_level != current_system_level) {
                current_system_level = system_level;
                system_level_changed = system_level;
            }
            set_disabled(userapp_level_changed, system_level_changed);
            const restart_localization = document.getElementById('restart_localization');
            const localize_status = data.localize_status?.at(-1)
            const text = localize_status == 2 ? '位置推定の再試行' : localize_status == 1 ? '位置推定中' : '位置推定状態不明';
            const disabled = localize_status != 2;
            if (restart_localization.textContent != text || restart_localization.disabled != disabled) {
                restart_localization.textContent = text;
                restart_localization.disabled = localize_status != 2;
            }
            document.body.classList.remove('disabled');
        })
        .catch(error => {
            console.error('Error:', error);
            document.body.classList.add('disabled');
        });

}

function handle_camera_image() {
    if (document.getElementById('image-items').style.display == 'none') {
        return;
    }
    fetch('/camera_image/', {})
        .then(response => response.json())
        .then(data => {
            for (d of data) {
                const img = document.getElementById(`camera_${d.position}_image`);
                img.src = d.image ?? '';
                img.style.transform = d.transform ?? '';
            }
        })
        .catch(error => console.error('Error:', error));
}

function handle_custom_image() {
    if (document.getElementById('image-items').style.display == 'none') {
        return;
    }
    fetch('/custom_image/', {})
        .then(response => response.json())
        .then(data => {
            const img = document.getElementById('custom_image');
            img.src = data.image ?? '';
            img.style.transform = data.transform ?? '';
        })
        .catch(error => console.error('Error:', error));
}

function init_selector() {
    const button = document.getElementById('selector-btn');
    const container = document.getElementById('selector-container');
    const backdrop = document.getElementById('selector-backdrop');

    button.addEventListener('click', (e) => {
        e.stopPropagation();
        const open = container.classList.toggle('open');
        backdrop.style.display = open ? 'block' : 'none';
    });

    backdrop.addEventListener('click', () => {
        container.classList.remove('open');
        backdrop.style.display = 'none';
    });

    container.addEventListener('click', (e) => {
        e.stopPropagation();
    });

    document.querySelectorAll("#settings-container > fieldset").forEach(el => {
        const legend = el.querySelector('legend');
        if (!legend) return;

        const title = legend.textContent;

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.checked = el.style.display !== 'none';

        checkbox.addEventListener('change', () => {
            el.style.display = checkbox.checked ? 'block' : 'none';
        });

        const label = document.createElement('label');
        label.style.display = 'block';

        label.appendChild(checkbox);
        label.appendChild(document.createTextNode(` ${title}`));
        container.appendChild(label);
    });
}

document.addEventListener('DOMContentLoaded', function () {

    add_destination_dialog = document.getElementById('add_destination_dialog');
    add_destination_dialog.addEventListener('close', add_destination_close);

    generic_confirm_dialog = document.getElementById('generic_confirm_dialog');
    generic_confirm_dialog.addEventListener('close', generic_confirm_close);

    fetch('/directory/', {})
        .then(response => response.json())
        .then(data => {
            console.log(data);
            if (!data.tours) {
                alert('ただいま準備中です。後で再読み込みしてください。');
                return;
            }
            directory_data = data;
            __debug__.data_timer = setInterval(handle_last_data, 1000);
            __debug__.camera_timer = setInterval(handle_camera_image, 2000);
            __debug__.image_timer = setInterval(handle_custom_image, 1000);
        })
        .catch(error => {
            console.error('Error:', error);
            alert(`地図データの取得に失敗しました。Error: ${error}`);
        });

    init_selector();
});
