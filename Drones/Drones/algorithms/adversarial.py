from __future__ import annotations

import random
from typing import TYPE_CHECKING
from abc import ABC, abstractmethod

import algorithms.evaluation as evaluation
from world.game import Agent, Directions

if TYPE_CHECKING:
    from world.game_state import GameState


class MultiAgentSearchAgent(Agent, ABC):
    """
    Base class for multi-agent search agents (Minimax, AlphaBeta, Expectimax).
    """

    def __init__(self, depth: str = "2", _index: int = 0, prob: str = "0.0") -> None:
        self.index = 0  # Drone is always agent 0
        self.depth = int(depth)
        self.prob = float(
            prob
        )  # Probability that each hunter acts randomly (0=greedy, 1=random)
        self.evaluation_function = evaluation.evaluation_function

    @abstractmethod
    def get_action(self, state: GameState) -> Directions | None:
        """
        Returns the best action for the drone from the current GameState.
        """
        pass


class RandomAgent(MultiAgentSearchAgent):
    """
    Agent that chooses a legal action uniformly at random.
    """

    def get_action(self, state: GameState) -> Directions | None:
        """
        Get a random legal action for the drone.
        """
        legal_actions = state.get_legal_actions(self.index)
        return random.choice(legal_actions) if legal_actions else None


class MinimaxAgent(MultiAgentSearchAgent):
    """
    Minimax agent for the drone (MAX) vs hunters (MIN) game.
    """

    def get_action(self, state: GameState) -> Directions | None:
        """
        Returns the best action for the drone using minimax.

        Tips:
        - The game tree alternates: drone (MAX) -> hunter1 (MIN) -> hunter2 (MIN) -> ... -> drone (MAX) -> ...
        - Use self.depth to control the search depth. depth=1 means the drone moves once and each hunter moves once.
        - Use state.get_legal_actions(agent_index) to get legal actions for a specific agent.
        - Use state.generate_successor(agent_index, action) to get the successor state after an action.
        - Use state.is_win() and state.is_lose() to check terminal states.
        - Use state.get_num_agents() to get the total number of agents.
        - Use self.evaluation_function(state) to evaluate leaf/terminal states.
        - The next agent is (agent_index + 1) % num_agents. Depth decreases after all agents have moved (full ply).
        - Return the ACTION (not the value) that maximizes the minimax value for the drone.
        """
        # TODO: Implement your code here
        num_agents = state.get_num_agents()

        def minimax(game_state, agent_index, depth):

            # estado terminal
            if game_state.is_win() or game_state.is_lose() or depth == self.depth:
                return self.evaluation_function(game_state)

            actions = game_state.get_legal_actions(agent_index)

            if not actions:
                return self.evaluation_function(game_state)

            next_agent = (agent_index + 1) % num_agents
            next_depth = depth + 1 if next_agent == 0 else depth

            # DRONE (MAX)
            if agent_index == 0:

                value = float("-inf")

                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)

                    value = max(
                        value,
                        minimax(successor, next_agent, next_depth)
                    )

                return value

            #HUNTERS (MIN)
            else:

                value = float("inf")

                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)

                    value = min(
                        value,
                        minimax(successor, next_agent, next_depth)
                    )

                return value

        # elegir mejor acción para el dron 

        best_action = None
        best_value = float("-inf")

        for action in state.get_legal_actions(0):

            successor = state.generate_successor(0, action)

            value = minimax(successor, 1, 0)

            if value > best_value:
                best_value = value
                best_action = action

        return best_action


class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Alpha-Beta pruning agent. Same as Minimax but with alpha-beta pruning.
    MAX node: prune when value > beta (strict).
    MIN node: prune when value < alpha (strict).
    """

    def get_action(self, state: GameState) -> Directions | None:
        """
        Returns the best action for the drone using alpha-beta pruning.

        Tips:
        - Same structure as MinimaxAgent, but with alpha-beta pruning.
        - Alpha: best value MAX can guarantee (initially -inf).
        - Beta: best value MIN can guarantee (initially +inf).
        - MAX node: prune when value > beta (strict inequality, do NOT prune on equality).
        - MIN node: prune when value < alpha (strict inequality, do NOT prune on equality).
        - Update alpha at MAX nodes: alpha = max(alpha, value).
        - Update beta at MIN nodes: beta = min(beta, value).
        - Pass alpha and beta through the recursive calls.
        """
        # -- VERSIÓN INICIAL ---------------------------------
        # Seguimos la misma estructura de Minimax.
        # Aquí usamos alpha y beta para evitar explorar ramas del árbol 
        # que ya sabemos que no van a afectar la decisión final.

        # Si nodo es MAX:
        # si valor actual supera beta, el nodo MIN padre
        # nunca escogería este camino, entonces no tiene sentido seguir
        # explorando los demás hijos y se corta la búsqueda (break).

        # Si nodo es MIN:
        # si valor actual es menor que alpha, el nodo MAX
        # padre tampoco elegiría este camino, así que también se puede cortar
        # la exploración de los hijos restantes.
        # -------------------------------------------------------------


        num_agents = state.get_num_agents()
 
        def alphabeta(game_state, agent_index, depth, alpha, beta):
            # estado terminal o profundidad máxima
            if game_state.is_win() or game_state.is_lose() or depth == self.depth:
                return self.evaluation_function(game_state)
 
            actions = game_state.get_legal_actions(agent_index)
 
            if not actions:
                return self.evaluation_function(game_state)
 
            next_agent = (agent_index + 1) % num_agents
            # La profundidad aumenta solo cuando todos los agentes ya movieron
            # (es decir, cuando volvemos al agente 0 = dron)
            next_depth = depth + 1 if next_agent == 0 else depth
 
            # DRON: nodo MAX
            if agent_index == 0:
                value = float("-inf")
 
                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)
                    value = max(value, alphabeta(successor, next_agent, next_depth, alpha, beta))
 
                    # Poda: MIN no permite que MAX tenga más que beta
                    if value > beta:
                        break
 
                    # Actualizar alpha
                    alpha = max(alpha, value)
 
                return value
 
            # CAZADORES: nodos MIN 
            else:
                value = float("inf")
 
                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)
                    value = min(value, alphabeta(successor, next_agent, next_depth, alpha, beta))
 
                    # Poda: MAX no elige algo menor que alpha.
                    if value < alpha:
                        break
 
                    # Actualizar beta
                    beta = min(beta, value)
 
                return value
 
        #  Raíz
        best_action = None
        best_value = float("-inf")
        alpha = float("-inf")
        beta = float("inf")
 
        for action in state.get_legal_actions(0):
            successor = state.generate_successor(0, action)
            value = alphabeta(successor, 1, 0, alpha, beta)
 
            if value > best_value:
                best_value = value
                best_action = action
 
            alpha = max(alpha, best_value)
 
        return best_action


class ExpectimaxAgent(MultiAgentSearchAgent):
    """
    Expectimax agent with a mixed hunter model.

    Each hunter acts randomly with probability self.prob and greedily
    (worst-case / MIN) with probability 1 - self.prob.

    * When prob = 0:  behaves like Minimax (hunters always play optimally).
    * When prob = 1:  pure expectimax (hunters always play uniformly at random).
    * When 0 < prob < 1: weighted combination that correctly models the
      actual MixedHunterAgent used at game-play time.

    Chance node formula:
        value = (1 - p) * min(child_values) + p * mean(child_values)
    """

    def get_action(self, state: GameState) -> Directions | None:
        """
        Returns the best action for the drone using expectimax with mixed hunter model.

        Tips:
        - Drone nodes are MAX (same as Minimax).
        - Hunter nodes are CHANCE with mixed model: the hunter acts greedily with
          probability (1 - self.prob) and uniformly at random with probability self.prob.
        - Mixed expected value = (1-p) * min(child_values) + p * mean(child_values).
        - When p=0 this reduces to Minimax; when p=1 it is pure uniform expectimax.
        - Do NOT prune in expectimax (unlike alpha-beta).
        - self.prob is set via the constructor argument prob.
        """
        # -- VERSIÓN INICIAL -----------------------
        # Misma  estructura que en Minimax: calculamos valor esperado porque el 
        # cazador no siempre actúa de forma óptima.

        # Por cada acción se calcula el valor del estado sucesor, luego se combinan los dos comportamientos:
        # 1) con probabilidad (1 - p) -> cazador actúa de forma greedy, tomamos mínimo valor
        # 2) con probabilidad p elige una acción al azar -> se toma el promedio de los valores

        #  valor final: valor = (1 - p) * min(child_values) + p * promedio(child_values)

        # Si p = 0 → el cazador siempre es greedy → equivalente a Minimax.
        # Si p = 1 → el cazador siempre es aleatorio → expectimax puro.
        # Si 0 < p < 1 → mezcla de comportamiento óptimo y aleatorio.
        # -------------------------------------------------------------

        num_agents = state.get_num_agents()
        p = self.prob  # probabilidad de que cada cazador actúe al azar

        def expectimax(game_state, agent_index, depth):
            # caso terminal o profundidad máxima
            if game_state.is_win() or game_state.is_lose() or depth == self.depth:
                return self.evaluation_function(game_state)

            actions = game_state.get_legal_actions(agent_index)

            if not actions:
                return self.evaluation_function(game_state)

            next_agent = (agent_index + 1) % num_agents
            next_depth = depth + 1 if next_agent == 0 else depth

            # DRON: nodo MAX 
            if agent_index == 0:
                value = float("-inf")

                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)
                    value = max(value, expectimax(successor, next_agent, next_depth))

                return value

            # CAZADORES: nodo AZAR
            else:
                # Calculamos el valor de cada hijo sin podas
                child_values = []
                for action in actions:
                    successor = game_state.generate_successor(agent_index, action)
                    child_values.append(expectimax(successor, next_agent, next_depth))

                # Comportamiento greedy
                greedy_value = min(child_values)

                # comportamiento aleatorio
                random_value = sum(child_values) / len(child_values)

                # combinación p
                return (1 - p) * greedy_value + p * random_value

        # elegir mejor acción para el dron en raíz
        best_action = None
        best_value = float("-inf")

        for action in state.get_legal_actions(0):
            successor = state.generate_successor(0, action)
            value = expectimax(successor, 1, 0)

            if value > best_value:
                best_value = value
                best_action = action

        return best_action
 
