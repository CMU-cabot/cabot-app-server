#!/usr/bin/env python

# Copyright (c) 2025  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import asyncio
import httpx
import logging
import os


logging.basicConfig(format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("agent")
logger.setLevel(logging.INFO)

PUBLIC = os.getenv("CABOT_WEBUI_PUBLIC")
LOCAL = os.getenv("CABOT_WEBUI_LOCAL", "http://localhost:5000")
CABOT_NAME = os.getenv("CABOT_NAME", "UNKNOWN")

WORKERS = 4


def filter_headers(headers: dict) -> dict:
    return {k: v for k, v in headers.items() if k.lower().startswith("content-")}


async def handle_request(
    client: httpx.AsyncClient,
    req: dict,
):
    try:
        async with client.stream(
            req["method"],
            LOCAL + req["path"],
            params=req["query"],
            headers=req["headers"],
            content=req["body"],
        ) as resp:
            status = resp.status_code
            headers = dict(resp.headers)
            body = b"".join([chunk async for chunk in resp.aiter_raw()])

        await client.post(
            f"{PUBLIC}/_response/{req['id']}",
            content=body,
            headers={
                "X-Status": str(status),
                **filter_headers(headers),
            },
        )

    except Exception as e:
        logger.error(f"[handle] error req_id={req.get('id')} " f"path={req.get('path')}: {repr(e)}")
        try:
            await client.post(
                f"{PUBLIC}/_response/{req['id']}",
                headers={"X-Status": "502"},
                content=b"agent error",
            )
        except Exception:
            pass


async def fetch_worker(
    worker_id: int,
    client: httpx.AsyncClient,
):
    next_timeout = httpx.Timeout(
        connect=1.0,
        read=30.0,
        write=30.0,
        pool=5.0,
    )
    while True:
        try:
            r = await client.get(f"{PUBLIC}/_next", timeout=next_timeout)
            if r.status_code == 204:
                continue

            r.raise_for_status()
            req = r.json()

            await handle_request(client, req)

        except httpx.ConnectError:
            await asyncio.sleep(1)

        except Exception as e:
            logger.error(f"[worker {worker_id}] error: {PUBLIC} {repr(e)}")
            await asyncio.sleep(1)


async def run():
    timeout = httpx.Timeout(
        connect=10.0,
        read=30.0,
        write=30.0,
        pool=5.0,
    )

    async with httpx.AsyncClient(timeout=timeout, headers={"X-CaBot-Name": CABOT_NAME}) as client:
        workers = [asyncio.create_task(fetch_worker(i, client)) for i in range(WORKERS)]
        await asyncio.gather(*workers)


if __name__ == "__main__":
    if PUBLIC:
        logger.info(f"Starting WebUI agent for {PUBLIC}")
        asyncio.run(run())
