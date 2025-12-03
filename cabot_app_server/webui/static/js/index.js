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

function renderSections(sections) {
    let html = "";
    for (const section of sections) {
        if (section.items) {
            html += `<fieldset><legend>${section.title}</legend>`;
            for (const item of section.items) {
                if (item.content?.sections) {
                    html += `<fieldset><legend>${item.title}</legend>`;
                    html += renderSections(item.content.sections);
                    html += `</fieldset>`;
                } else {
                    html += `<div data-node="${item.nodeID}" data-demo="${item.forDemonstration ?? false}">${item.title ?? 'Untitled'}</div>`;
                }
            }
            html += `</fieldset>`;
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
    fetch('/directory/', {})
        .then(response => response.json())
        .then(data => {
            console.log(data);
            const lang = 'ja'
            document.getElementById('destinations').innerHTML = renderSections(data.sections[lang]);
            document.getElementById('tours').innerHTML = renderTours(data.tours, lang);
        })
        .catch(error => console.error('Error:', error));
});
