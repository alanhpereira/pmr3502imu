#!/usr/bin/env python3

'''
	Author: Thiago Martins.
'''
import logging
import aiohttp
import asyncio
from asyncio import Queue
from aiohttp import web, MultipartWriter
import time
import sys
import os
import argparse
import numpy as np
import rumo

class ImuClient():
    def __init__(self, address, port, datacallback):
        self._server = address
        self._port = port
        self._datacallback = datacallback
        self._keepRunning = True
        self._read = None

    def start(self):
        pass

    def stop(self):
        print("Encerrando...")
        self._keepRunning = False
        if self._writer:
            self._writer.close()
            self._reader = None
            self._writer = None

    def started(self):
        return self._keepRunning

    def __del__(self):
        pass

    async def maintask(self):
        print("Abrindo conexão ao serviço IMU... ")
        try:
            self._reader, self._writer = await asyncio.open_connection(self._server, self._port)
        except OSError as e:
            print("Conexão com serviço IMU falhou.")
            self._keepRunning = False
        else:
            print("Conexão com serviço IMU estabelecida.")
        while self._keepRunning:
            data = await self._reader.read(32)
            if len(data)==32:
                self._datacallback(data)
            else:
                self.stop()



class ServicoRumo():

    def __init__(self, app, endereco_servidor, porta_servidor, endereco_imu, porta_imu, estimador):
        self._app = app
        self._est = rumo.cria_estimador_rumo(time.clock_gettime(time.CLOCK_REALTIME), 0, 40, self._newest, estimador)
        self._endereco_servidor = endereco_servidor
        self._porta_servidor = porta_servidor
        self._app['app_object'] = self
        # Tarefas de inicializacao e encerramento
        self._app.on_startup.append(self._inicializa_tarefas)
        self._app.on_cleanup.append(self._encerra_tarefas)
        self._app.router.add_routes([web.get('/wsctrl', self._websocket_handler)])
        self._app.router.add_routes([web.get('/', self._raiz)])

        STATIC_PATH = os.path.join(os.path.dirname(__file__), "static")
        self._app.router.add_static('/static/', STATIC_PATH, name='static')
        self._keep_alive = True
        self._worker_task = None
        self._connections = set()
        self._clienteIMU = ImuClient(endereco_imu, porta_imu, self._est.callback)

    def run(self):
        web.run_app(self._app, host=self._endereco_servidor, port=self._porta_servidor, shutdown_timeout=0.2)

    def _newest(self, t, theta, sigma, omega):
        for connection in self._connections:
            connection.put_nowait((t,theta,sigma,omega))

    async def _raiz(self, request):
        raise web.HTTPFound('./static/showbearing.html')

    async def _inicializa_tarefas(self, app):
        self._worker_task = asyncio.create_task(self._clienteIMU.maintask())

    async def _encerra_tarefas(self, app):
        self._keep_alive = False
        self._clienteIMU.stop()
        if self._worker_task is not None:
            self._worker_task.cancel()
            await self._worker_task
            self._worker_task = None

    # Responde a uma conexão web socket
    async def _websocket_handler(self, request):
        print("Connection established!")

        messages = Queue()
        self._connections.add(messages)
        ws = web.WebSocketResponse(receive_timeout=0)
        await ws.prepare(request)
        try:
            while self._keep_alive and not ws.closed:
                try:
                    from_client = await ws.receive(timeout = 0)
                    if from_client.type==web.WSMsgType.CLOSE:
                        break
                    elif from_client.type==web.WSMsgType.TEXT:
                        pass
                except asyncio.TimeoutError as e:
                    pass
                msg = None
                # Esvazia a fila, manda só a previsão mais recente
                while msg is None or not messages.empty():
                    msg = await messages.get()
                    messages.task_done()

                await ws.send_json(msg)
        except Exception as e:
            print("ERROR")
            print(e.__class__.__qualname__)
        self._connections.remove(messages)

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-e', help="Endereço externo do servidor")
    parser.add_argument('-p', help="Porta do servidor", default="8087")
    parser.add_argument('-i', help="Numero da porta IMU", default="1234")
    parser.add_argument('-s', help="Endereço do serviço IMU", default="127.0.0.1")
    parser.add_argument('-f', help="Nome do estimador", default="bussola")

    args = parser.parse_args()
    endereco_servidor = args.e
    porta_servico = int(args.p)
    porta_imu = int(args.i)
    endereco_imu = args.s
    estimador = args.f

    if endereco_servidor == None:
        endereco_servidor = "0.0.0.0"

    print("Endereço do servidor: " + endereco_servidor)
    print("Porta do servidor: " + str(porta_servico))

    print(f"Usando servico de IMU em {endereco_imu}:{porta_imu}")
    print(f"Estimador de rumo: {estimador}")

    serviceObj = ServicoRumo(web.Application(), endereco_servidor, porta_servico, endereco_imu, porta_imu, estimador)

    serviceObj.run()

    return 0


if __name__ == '__main__':
    sys.exit(main())
