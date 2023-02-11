from re import template
from fastapi import FastAPI, WebSocket, Request
import serial
from fastapi.responses import HTMLResponse
from jinja2 import Environment, FileSystemLoader
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import socket


ser = serial.Serial('COM10', baudrate=115200, timeout=1)
app = FastAPI()

# Configure the Jinja2 template engine
templates = Environment(
    loader=FileSystemLoader("templates"),
    autoescape=False
)

app.mount("/static", StaticFiles(directory="static"), name="static")


templates = Jinja2Templates(directory="templates")


@app.get("/")
async def read_item(request: Request):
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
    context = {'request': request, "host_ip": host_ip}
    return templates.TemplateResponse("index.html", context=context)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        print(data)
        ser.write(data.encode())
