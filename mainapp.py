from re import template
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import serial
from fastapi.responses import HTMLResponse
from jinja2 import Environment, FileSystemLoader

ser = serial.Serial('COM10', baudrate=115200, timeout=1)
app = FastAPI()

# Configure the Jinja2 template engine
templates = Environment(
    loader=FileSystemLoader("templates"),
    autoescape=False
)


@app.get("/")
async def index(name: str = "World"):
    template = templates.get_template("index.html")
    content = template.render(name=name)
    return HTMLResponse(content)


@app.get("/")
async def get(request):
    return templates.TemplateResponse('index.html', {"request": request})


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        print(data)
        ser.write(data.encode())