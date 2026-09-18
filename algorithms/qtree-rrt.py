#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import math
import random
import time
from pathlib import Path

import numpy as np
from PIL import Image

# CONFIGURAÇÕES DO MAPA E PARÂMETROS
BASE_DIR = Path(__file__).resolve().parent
DATASET_DIR = BASE_DIR / "DATASET"
OBSTACLE_THRESHOLD = 127

# Parâmetros da Quadtree e Amostragem
MIN_SIZE = 9
LIMITE_MINIMO = int(1.4 * MIN_SIZE)
DIST_MAX_CENTROS = 60
MARGEM_OBSTACULO = 12
QTD_PONTOS_RRT = 100
DIST_CONEXAO_RRT = 60
SEED = 42
ponto_inicial = (26, 26)
ponto_final = (384, 384)

# ALGORITMO: QUADTREE + AMOSTRAGEM REGIONAL
class QuadtreeRegionalPlanner:

    def __init__(self, start, goal, map_array):
        self.map_array = map_array
        self.h_img, self.w_img = map_array.shape

        self.start_pt = tuple(start)
        self.goal_pt = tuple(goal)

        # Estruturas
        self.retangulos = []
        self.centros = []
        self.quadrantes_canal_seguro = []
        self.pontos_rrt = []
        self.arestas_qtree = []
        self.arestas_rrt = []
        self.caminho_macro = []

        # Área segura
        self.area_segura = self.gerar_area_segura()

    # ÁREAS PROMISSORAS
    def gerar_area_segura(self):
        binaria_inv = (self.map_array < OBSTACLE_THRESHOLD).astype(np.uint8)
        k = MARGEM_OBSTACULO
        dilatada = np.zeros_like(binaria_inv)

        for dy in range(-k // 2, k // 2 + 1):
            for dx in range(-k // 2, k // 2 + 1):
                dilatada = np.maximum(
                    dilatada,
                    np.roll(np.roll(binaria_inv, dx, axis=1), dy, axis=0)
                )

        return (dilatada == 0).astype(np.uint8) * 255

    # QUADTREE
    def is_bloco_branco(self, x, y, w, h):
        return np.all(
            self.map_array[y:y + h, x:x + w] >= OBSTACLE_THRESHOLD
        )

    def aplicar_quadtree(self, x, y, w, h):
        if (
            w <= MIN_SIZE or
            h <= MIN_SIZE or
            self.is_bloco_branco(x, y, w, h)
        ):
            if (
                self.is_bloco_branco(x, y, w, h)
                and w >= LIMITE_MINIMO
                and h >= LIMITE_MINIMO
            ):
                self.retangulos.append((x, y, w, h))
                self.centros.append(
                    (x + w // 2, y + h // 2)
                )
            return

        w_esq = w // 2
        w_dir = w - w_esq
        h_topo = h // 2
        h_fundo = h - h_topo

        self.aplicar_quadtree(x, y, w_esq, h_topo)
        self.aplicar_quadtree(x + w_esq, y, w_dir, h_topo)
        self.aplicar_quadtree(x, y + h_topo, w_esq, h_fundo)
        self.aplicar_quadtree(x + w_esq, y + h_topo, w_dir, h_fundo)

    # COLISÃO
    def linha_livre(self, p1, p2):
        x0, y0 = int(p1[0]), int(p1[1])
        x1, y1 = int(p2[0]), int(p2[1])

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if (
                x0 < 0 or
                x0 >= self.w_img or
                y0 < 0 or
                y0 >= self.h_img
            ):
                return False

            if self.map_array[y0, x0] < OBSTACLE_THRESHOLD:
                return False

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True

    # FUNÇÕES AUXILIARES
    def ponto_em_area_segura(self, ponto):
        x, y = int(ponto[0]), int(ponto[1])
        if 0 <= x < self.w_img and 0 <= y < self.h_img:
            return self.area_segura[y, x] == 255
        return False

    def ponto_em_quadrantes(self, ponto, quadrantes):
        px, py = ponto
        for x, y, w, h in quadrantes:
            if x <= px <= x + w and y <= py <= y + h:
                return True
        return False

    # GRAFO QUADTREE
    def construir_grafo_qtree(self):
        adj = {
            tuple(c): []
            for c in self.centros
            if self.ponto_em_area_segura(c)
        }

        lista_centros_validos = list(adj.keys())

        for i, c1 in enumerate(lista_centros_validos):
            for j, c2 in enumerate(lista_centros_validos):
                if i >= j:
                    continue

                d = math.hypot(
                    c1[0] - c2[0],
                    c1[1] - c2[1]
                )

                if d <= DIST_MAX_CENTROS and self.linha_livre(c1, c2):
                    adj[c1].append((c2, d))
                    adj[c2].append((c1, d))
                    self.arestas_qtree.append((c1, c2))

        return adj

    # DIJKSTRA
    def dijkstra(self, grafo, origem, destino):
        pq = [(0.0, origem)]
        distancias = {origem: 0.0}
        anteriores = {}
        visitados = set()

        while pq:
            custo_atual, atual = heapq.heappop(pq)

            if atual in visitados:
                continue

            visitados.add(atual)

            if atual == destino:
                caminho = [atual]
                while atual in anteriores:
                    atual = anteriores[atual]
                    caminho.append(atual)
                return caminho[::-1], custo_atual, len(visitados)

            for vizinho, peso in grafo.get(atual, []):
                novo_custo = custo_atual + peso
                if novo_custo < distancias.get(vizinho, float("inf")):
                    distancias[vizinho] = novo_custo
                    anteriores[vizinho] = atual
                    heapq.heappush(
                        pq,
                        (novo_custo, vizinho)
                    )

        return [], 0.0, len(visitados)

    # PLANNING
    def planning(self):
        start_qtree_time = time.time()

        self.aplicar_quadtree(
            0,
            0,
            self.w_img,
            self.h_img
        )

        if not self.centros:
            return None, time.time() - start_qtree_time, 0.0, 0, 0, 0

        grafo_qtree = self.construir_grafo_qtree()

        if not grafo_qtree.keys():
            return None, time.time() - start_qtree_time, 0.0, len(self.retangulos), 0, 0

        inicio_centro = min(
            grafo_qtree.keys(),
            key=lambda c: math.hypot(
                self.start_pt[0] - c[0],
                self.start_pt[1] - c[1]
            )
        )

        fim_centro = min(
            grafo_qtree.keys(),
            key=lambda c: math.hypot(
                self.goal_pt[0] - c[0],
                self.goal_pt[1] - c[1]
            )
        )

        self.caminho_macro, _, visitados_macro = self.dijkstra(
            grafo_qtree,
            inicio_centro,
            fim_centro
        )

        tempo_fase_qtree = time.time() - start_qtree_time
        start_refinamento_time = time.time()

        if not self.caminho_macro:
            return None, tempo_fase_qtree, 0.0, len(self.retangulos), 0, visitados_macro

        # CANAL SEGURO
        set_caminho_macro = set(self.caminho_macro)

        for (x, y, w, h) in self.retangulos:
            centro = (x + w // 2, y + h // 2)
            if centro in set_caminho_macro:
                self.quadrantes_canal_seguro.append((x, y, w, h))

        # AMOSTRAGEM RRT
        num_quadrantes = len(self.quadrantes_canal_seguro)
        pontos_por_bloco = max(
            1,
            QTD_PONTOS_RRT // max(1, num_quadrantes)
        )

        for q in self.quadrantes_canal_seguro:
            x, y, w, h = q
            for _ in range(pontos_por_bloco):
                px = random.randint(x, x + w - 1)
                py = random.randint(y, y + h - 1)
                ponto = (px, py)
                if self.ponto_em_area_segura(ponto):
                    self.pontos_rrt.append(ponto)

        self.pontos_rrt.append(self.caminho_macro[0])
        self.pontos_rrt.append(self.caminho_macro[-1])
        self.pontos_rrt = list(set(self.pontos_rrt))

        # GRAFO RRT
        grafo_rrt = {p: [] for p in self.pontos_rrt}

        for i, p1 in enumerate(self.pontos_rrt):
            for j, p2 in enumerate(self.pontos_rrt):
                if i >= j:
                    continue

                d = math.hypot(
                    p1[0] - p2[0],
                    p1[1] - p2[1]
                )

                if d <= DIST_CONEXAO_RRT:
                    if (
                        self.ponto_em_quadrantes(
                            p1,
                            self.quadrantes_canal_seguro
                        )
                        and
                        self.ponto_em_quadrantes(
                            p2,
                            self.quadrantes_canal_seguro
                        )
                        and
                        self.linha_livre(p1, p2)
                    ):
                        grafo_rrt[p1].append((p2, d))
                        grafo_rrt[p2].append((p1, d))
                        self.arestas_rrt.append((p1, p2))

        caminho_final, _, visitados_rrt = self.dijkstra(
            grafo_rrt,
            self.caminho_macro[0],
            self.caminho_macro[-1]
        )

        tempo_fase_refinamento = time.time() - start_refinamento_time

        return (
            caminho_final,
            tempo_fase_qtree,
            tempo_fase_refinamento,
            len(self.retangulos),
            len(self.quadrantes_canal_seguro),
            visitados_macro + visitados_rrt
        )


# EXEMPLO DE EXECUÇÃO UNITÁRIA
if __name__ == "__main__":
    random.seed(SEED)
    np.random.seed(SEED)

    # Teste com o primeiro mapa como exemplo
    mapa_exemplo = DATASET_DIR / "mapa01.png"

    if mapa_exemplo.exists():
        print(f"[INFO] Executando teste unitário em: {mapa_exemplo.name}")
        img_pil = Image.open(mapa_exemplo).convert("L")
        map_array = np.array(img_pil)

        planner = QuadtreeRegionalPlanner(
            ponto_inicial,
            ponto_final,
            map_array
        )

        path, t_qtree, t_ref, n_rects, n_seguros, visitados = planner.planning()

        if path:
            custo = sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) for i in range(len(path)-1))
            
            # Contagens separadas de pontos e arestas
            pontos_qtree_validos = len([c for c in planner.centros if planner.ponto_em_area_segura(c)])
            pontos_rrt_gerados = len(planner.pontos_rrt)
            total_vertices = pontos_qtree_validos + pontos_rrt_gerados
            
            total_arestas = len(planner.arestas_qtree) + len(planner.arestas_rrt)

            print(f"Sucesso! Caminho encontrado com {len(path)} waypoints e custo {custo:.2f}.")
            print(f"Pontos gerados pela Quadtree (centros válidos): {pontos_qtree_validos}")
            print(f"Pontos gerados pela Amostragem RRT: {pontos_rrt_gerados}")
            print(f"Total geral de vértices (QTree + RRT): {total_vertices}")
            print(f"Total de arestas do grafo consolidado: {total_arestas}")
            print(f"Tempo QTree: {t_qtree:.4f}s | Tempo Refinamento: {t_ref:.4f}s (Total: {t_qtree + t_ref:.4f}s)")
        else:
            print("Falha ao encontrar caminho.")
    else:
        print(f"[AVISO] Arquivo {mapa_exemplo} não encontrado para teste.")
