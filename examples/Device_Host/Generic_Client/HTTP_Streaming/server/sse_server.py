from flask import Flask, Response
import time
import json
import random

app = Flask(__name__)

def generate_events():
    while True:
        # Simulate dynamic data
        event_id = random.randint(1, 1000)
        news = {
            "id": event_id,
            "title": f"title {event_id}",
            "content": f"content {event_id}"
        }

        # Format SSE message
        yield f"id: {event_id}\ndata: {json.dumps({'news': [news]})}\n\n"

        time.sleep(2)  # send every 2 seconds

@app.route("/stream")
def stream():
    return Response(generate_events(), mimetype="text/event-stream")

if __name__ == "__main__":
    app.run(debug=True, threaded=True)
