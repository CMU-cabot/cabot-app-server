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
let add_destination_dialog;
let generic_confirm_dialog;

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
                    html += `<fieldset class="collapsed"><legend onclick="toggleBox(this)">${item.title}</legend>`;
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

function renderCurrentDestinations(data) {
    let html = "";
    const tour = data['share.Tour']?.at(-1);
    let skip = true;
    if (tour) {
        if (tour.currentDestination) {
            const name = destination_name(tour.currentDestination);
            html += `<div>${name} <button onclick="skip('${tour.currentDestination}')">スキップ</button></div>`;
            skip = false;
        }
        for (const destination of tour.destinations ?? []) {
            const name = destination_name(destination);
            html += `<div>${name}`;
            if (skip) {
                html += ` <button onclick="skip('${destination}')">スキップ</button>`;
                skip = false;
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

function renderSpeakHistories(data) {
    let html = '';
    for (const text of data['share.Speak'] ?? []) {
        html += `<div>${text}</div>`;
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

function renderDestinationHistories(data) {
    let html = "";
    const arr = data['destination'] ?? [];
    const destinations = arr.filter((v, i) => v !== arr[i - 1]);
    for (const destination of destinations) {
        let name
        switch (destination) {
            case '__arrived__':
                name = '到着済み';
                break;
            case '__cancel__':
                name = 'キャンセル済み';
                break;
            default:
                name = destination_name(destination);
                break;
        }
        html += `<div>${name}</div>`;
    }
    return html;
}

function get_touch_state(data) {
    let result = '';
    for (const touch of data['touch'] ?? []) {
        let touch_state;
        switch (touch.level) {
            case -1:
                touch_state = '未接続';
                break;
            case 0:
                touch_state = 'OFF';
                break;
            case 1:
                touch_state = 'ON';
                break;
            default:
                return `予期せぬデータ：${JSON.stringify(touch)}`;
                break;
        }
        if (result && result != touch_state) {
            return '判定中';
        }
        result = touch_state;
    }
    return result || '判定中';
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

function set_highlight(language_changed, voicerate_changed, handleside_changed, touchmode_changed, chatvisible_changed) {
    if (language_changed != undefined) {
        document.querySelectorAll('[data-language]').forEach(el => {
            if (el.dataset.language === language_changed) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (handleside_changed != undefined) {
        document.querySelectorAll('[data-handleside]').forEach(el => {
            if (el.dataset.handleside === handleside_changed) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (touchmode_changed != undefined) {
        document.querySelectorAll('[data-touchmode]').forEach(el => {
            if (el.dataset.touchmode === touchmode_changed) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (chatvisible_changed != undefined) {
        document.querySelectorAll('[data-chatvisible]').forEach(el => {
            if (el.dataset.chatvisible === chatvisible_changed) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
    if (voicerate_changed != undefined) {
        if (voicerate_changed < (0.2 + 0.35) / 2) {
            voicerate_changed = 'very-slow';
        } else if (voicerate_changed < (0.35 + 0.5) / 2) {
            voicerate_changed = 'slow';
        } else if (voicerate_changed < (0.5 + 0.65) / 2) {
            voicerate_changed = 'normal';
        } else if (voicerate_changed < (0.65 + 0.8) / 2) {
            voicerate_changed = 'fast';
        } else {
            voicerate_changed = 'very-fast';
        }
        document.querySelectorAll('[data-voicerate]').forEach(el => {
            if (el.dataset.voicerate === voicerate_changed) {
                el.classList.add('selected');
            } else {
                el.classList.remove('selected');
            }
        });
    }
}

document.addEventListener('DOMContentLoaded', function () {

    const urlParams = new URLSearchParams(window.location.search);
    document.getElementById('debug-info').style.display = urlParams.get('debug') == 'true' ? 'block' : 'none';

    add_destination_dialog = document.getElementById('add_destination_dialog');
    add_destination_dialog.addEventListener('close', add_destination_close);

    generic_confirm_dialog = document.getElementById('generic_confirm_dialog');
    generic_confirm_dialog.addEventListener('close', generic_confirm_close);

    fetch('/directory/', {})
        .then(response => response.json())
        .then(data => {
            console.log(data);
            directory_data = data;
        })
        .catch(error => console.error('Error:', error));

    __debug__.data_timer = setInterval(() => {
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
                replaceHTML('speak_histories', renderSpeakHistories(data));
                replaceHTML('chat_histories', renderChatHistories(data));
                replaceHTML('navigation_histories', renderDestinationHistories(data));
                replaceHTML('current_destinations', renderCurrentDestinations(data));
                replaceText('cabot_name', data['cabot_name']?.at(-1) ?? '未接続');
                replaceText('touch_state', get_touch_state(data));
                document.getElementById('messages').innerText = JSON.stringify(data, null, 2);
                let voicerate = data['share.ChangeUserVoiceRate']?.at(-1);
                if (voicerate && voicerate != current_voicerate) {
                    current_voicerate = voicerate;
                    voicerate_changed = voicerate;
                }
                let handleside = data['share.ChangeHandleSide']?.at(-1);
                if (handleside && handleside != current_handleside) {
                    current_handleside = handleside;
                    handleside_changed = handleside;
                }
                let touchmode = data['share.ChangeTouchMode']?.at(-1);
                if (touchmode && touchmode != cuurrent_touchmode) {
                    cuurrent_touchmode = touchmode;
                    touchmode_changed = touchmode;
                }
                let chatvisible = String(data['share.ChatStatus.visible']?.at(-1) ?? false);
                if (chatvisible != cuurrent_chatvisible) {
                    cuurrent_chatvisible = chatvisible;
                    chatvisible_changed = chatvisible;
                }
                set_highlight(language_changed, voicerate_changed, handleside_changed, touchmode_changed, chatvisible_changed);
            })
            .catch(error => console.error('Error:', error));
    }, 1000);

    __debug__.camera_timer = setInterval(() => {
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
    }, 1000);
    __debug__.image_timer = setInterval(() => {
        fetch('/custom_image/', {})
            .then(response => response.json())
            .then(data => {
                const img = document.getElementById('custom_image');
                img.src = data.image ?? '';
                img.style.transform = data.transform ?? '';
            })
            .catch(error => console.error('Error:', error));
    }, 1000);
});
