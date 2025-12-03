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
        flag1: false, flag2: false, location: 0, length: 0, info_id: new Date().getTime() * 1000000
    };
    post_data('/publish/', { event: 'share', data: JSON.stringify({ ...defaults, ...data }) });
}

function speak(data) {
    const defaults = {
        rate: 50, pitch: 50, volume: 50, lang: 'ja', voice: 'male', force: false, priority: 90, timeout: 2.0, channels: 3, request_id: new Date().getTime() * 1000000
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

function renderSections(sections, destinations, level = 0) {
    let html = "";
    for (const section of sections) {
        if (section.items) {
            if (level == 0) {
                html += `<fieldset><legend>${section.title}</legend>`;
            }
            for (const item of section.items) {
                if (item.content?.sections) {
                    html += `<fieldset class="collapsed"><legend onclick="toggleBox(this)">${item.title}</legend>`;
                    html += renderSections(item.content.sections, destinations, level + 1);
                    html += `</fieldset>`;
                } else {
                    if (item.nodeID) {
                        let nodeID = item.nodeID;
                        for (const dest of destinations) {
                            if (dest.value == nodeID) {
                                if ('arrivalAngle' in dest) {
                                    nodeID = `${nodeID}@${dest.arrivalAngle}`
                                }
                                break;
                            }
                        }
                        html += `<div data-demo="${item.forDemonstration ?? false}" onclick="add_destination('${nodeID}')">${item.title ?? 'Untitled'}</div>`;
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

function renderTours(tours, lang) {
    let html = "";
    for (const tour of tours) {
        html += `<div data-tour="${tour.tour_id}" data-debug="${tour.debug ?? false}">${tour['title-' + lang] ?? 'Untitled'}</div>`;
    }
    return html;
}

document.addEventListener('DOMContentLoaded', function () {

    setInterval(() => {
        fetch('/last_data/', {})
            .then(response => response.json())
            .then(data => {
                document.getElementById('messages').innerText = JSON.stringify(data, null, 2);
            })
            .catch(error => console.error('Error:', error));
    }, 1000);
    setInterval(() => {
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
    fetch('/directory/', {})
        .then(response => response.json())
        .then(data => {
            console.log(data);
            const lang = 'ja'
            document.getElementById('destinations').innerHTML = renderSections(data.sections[lang], data.destinations);
            document.getElementById('tours').innerHTML = renderTours(data.tours, lang);
        })
        .catch(error => console.error('Error:', error));
});
