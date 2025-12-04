let directory_data = {};
let node_names = {};
let __debug__ = {};
let current_lang = '';

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
    const clear = confirm('全部消してから追加しますか？');
    const first = clear || confirm('最初に追加しますか？');
    share({ type: 'OverrideDestination', value: node, flag1: clear, flag2: first });
}

function set_tour(tour) {
    if (confirm('ツアーを送信しますか？')) {
        share({ type: 'OverrideTour', value: tour });
    }
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
        html += `<div onclick="set_tour('${tour.tour_id}')">${tour['title-' + current_lang] ?? tour.tour_id}${tour.debug ? ' (Debug)' : ''}</div>`;
    }
    return html;
}

function clear_destinations(count) {
    if (confirm(`${count}個の目的地をキャンセルしますか？`)) {
        share({ type: 'ClearDestinations' });
    }
}

function destination_name(node) {
    return node_names[node.split("#")[0]] ?? node;
}

function skip(node) {
    if (confirm(`${destination_name(node)}をスキップしますか？`)) {
        share({ type: 'Skip', value: node });
    }
}

function renderCurrentDestinations(data) {
    let html = "";
    const tour = data['share.Tour']?.at(-1);
    let skip = true;
    if (tour) {
        let count = tour.currentDestination ? 1 : 0 + (tour.destinations ?? []).length;
        if (count > 0) {
            html += `<button onclick="clear_destinations(${count})">ナビゲーションを中止</button>`;
        }
        if (tour.currentDestination) {
            const name = destination_name(tour.currentDestination);
            html += `<div>${name}</div><button onclick="skip('${tour.currentDestination}')">${name}をスキップ</button><hr>`;
            skip = false;
        }
        for (const destination of tour.destinations ?? []) {
            const name = destination_name(destination);
            html += `<div>${name}</div>`;
            if (skip) {
                html += `<button onclick="skip('${destination}')">${name}をスキップ</button>`;
                skip = false;
            }
        }
    }
    return html;
}

function renderSpeakHistories(data) {
    let html = '';
    for (const text of data['share.Speak'] ?? []) {
        html += `<p>${text}</p>`;
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
}

document.addEventListener('DOMContentLoaded', function () {

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
                let lang = data['share.ChangeLanguage']?.at(-1);
                if (lang == 'zh-Hans') {
                    lang = 'zh-CN';
                }
                if (lang && lang != current_lang) {
                    console.log(`Switch to ${lang}`);
                    current_lang = lang;
                    document.getElementById('destinations').innerHTML = renderSections(directory_data.sections[current_lang]);
                    document.getElementById('tours').innerHTML = renderTours(directory_data.tours);
                    build_index();
                }
                document.getElementById('speak_histories').innerHTML = renderSpeakHistories(data);
                document.getElementById('chat_histories').innerHTML = renderChatHistories(data);
                document.getElementById('messages').innerText = JSON.stringify(data, null, 2);
                document.getElementById('current_destinations').innerHTML = renderCurrentDestinations(data);
            })
            .catch(error => console.error('Error:', error));
    }, 1000);

    __debug__.camera_timer = setInterval(() => {
        fetch('/camera_image/', {})
            .then(response => response.json())
            .then(data => {
                if (data.image) {
                    const img = document.getElementById('camera_image');
                    img.src = data.image;
                    img.style.transform = data.transform ?? '';
                }
            })
            .catch(error => console.error('Error:', error));
    }, 1000);
});
