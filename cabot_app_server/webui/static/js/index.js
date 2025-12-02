console.log("index.js loaded");

function post_data(url, data) {
    fetch(url, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            data: data
        })
    })
        .then(response => response.json())
        .catch(error => console.error('Error:', error));
}

function share(data) {
    const defaults = {
        flag1: false, flag2: false, location: 0, length: 0, info_id: new Date().getTime() * 1000000
    };
    post_data('/share/', JSON.stringify({ ...defaults, ...data }));
}

function manage(data) {
    post_data('/manage/', data);
}

document.addEventListener('DOMContentLoaded', function () {
    setInterval(() => {
        fetch('/last_data/', {})
            .then(response => response.json())
            .then(data => {
                console.log(data);
                document.getElementById("messages").innerText = JSON.stringify(data, null, 4);
            })
            .catch(error => console.error('Error:', error));
    }, 1000);
});
