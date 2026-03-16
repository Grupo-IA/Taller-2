from __future__ import annotations
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from algorithms.problems_csp import DroneAssignmentCSP


def backtracking_search(csp: DroneAssignmentCSP, assignment: dict[str, str] | None = None) -> dict[str, str] | None:
    """
    Basic backtracking search without optimizations.

    Tips:
    - An assignment is a dictionary mapping variables to values (e.g. {X1: Cell(1,2), X2: Cell(3,4)}).
    - Use csp.assign(var, value, assignment) to assign a value to a variable.
    - Use csp.unassign(var, assignment) to unassign a variable.
    - Use csp.is_consistent(var, value, assignment) to check if an assignment is consistent with the constraints.
    - Use csp.is_complete(assignment) to check if the assignment is complete (all variables assigned).
    - Use csp.get_unassigned_variables(assignment) to get a list of unassigned variables.
    - Use csp.domains[var] to get the list of possible values for a variable.
    - Use csp.get_neighbors(var) to get the list of variables that share a constraint with var.
    - Add logs to measure how good your implementation is (e.g. number of assignments, backtracks).

    You can find inspiration in the textbook's pseudocode:
    Artificial Intelligence: A Modern Approach (4th Edition) by Russell and Norvig, Chapter 5: Constraint Satisfaction Problems
    """
    if assignment is None:
        assignment = {}
    
    if csp.is_complete(assignment):
        return assignment
      
      
    var = csp.get_unassigned_variables(assignment)[0]  # Select the first unassigned variable
    
    for value in csp.domains[var]:  # Iterate over possible values
        if csp.is_consistent(var, value, assignment):  # Check consistency
            csp.assign(var, value, assignment)  # Assign the value
            result = backtracking_search(csp, assignment)  # Recur with the new assignment
            if result is not None:  # If a solution is found, return it
                return result
            csp.unassign(var, assignment)  # Backtrack if no solution found
            
    return None


def backtracking_fc(csp: "DroneAssignmentCSP", assignment=None, domains=None) -> dict[str, str] | None:
    """
    Backtracking search with Forward Checking.

    Tips:
    - Forward checking: After assigning a value to a variable, eliminate inconsistent values from
      the domains of unassigned neighbors. If any neighbor's domain becomes empty, backtrack immediately.
    - Save domains before forward checking so you can restore them on backtrack.
    - Use csp.get_neighbors(var) to get variables that share constraints with var.
    - Use csp.is_consistent(neighbor, val, assignment) to check if a value is still consistent.
    - Forward checking reduces the search space by detecting failures earlier than basic backtracking.
    """
    if assignment is None:
        assignment = {}
    if domains is None:
        domains = {var: list(csp.domains[var]) for var in csp.variables}
    
    if csp.is_complete(assignment):
        return assignment
      
    var = csp.get_unassigned_variables(assignment)[0]  # Select the first unassigned variable
    
    for value in domains[var]:  # Iterate over possible values
        if csp.is_consistent(var, value, assignment):  # Check consistency
            csp.assign(var, value, assignment)  # Assign the value
            
            # Forward checking: Save current domains to restore later
            saved_domains = {neighbor: list(domains[neighbor]) for neighbor in csp.get_neighbors(var)}
            
            # Eliminate inconsistent values from neighbors' domains
            for neighbor in csp.get_neighbors(var):
                if neighbor not in assignment:
                    domains[neighbor] = [val for val in domains[neighbor] if csp.is_consistent(neighbor, val, assignment)]
                    if not domains[neighbor]:  # If any neighbor's domain is empty, backtrack
                        break
            
            else:  # Only recurse if no neighbor's domain is empty
                result = backtracking_fc(csp, assignment, domains)  # Recur with the new assignment
                if result is not None:  # If a solution is found, return it
                    return result
            
            # Restore domains on backtrack
            for neighbor in saved_domains:
                domains[neighbor] = saved_domains[neighbor]
                
            csp.unassign(var, assignment)  # Backtrack if no solution found
    return None

def remove_inconsistent_values(csp: DroneAssignmentCSP, Ai, Aj):
    removido= False
    for dron in list(csp.domains[Ai]):
        posible= False
        j= 0
        while j < len(csp.domains[Aj]) and not posible:
            dron2= csp.domains[Aj][j]
            if csp.is_consistent(Ai, dron, {Aj: dron2}):
                posible= True
            j+= 1
        if posible == False:
            csp.domains[Ai].remove(dron)
            removido= True
    return removido
  
def AC_3(csp: DroneAssignmentCSP | None, arcos= None) -> bool:
    if arcos is not None:
        cola = list(arcos)
    else:
        cola = []
    for Ai in csp.variables:
      for Aj in csp.get_neighbors(Ai):
        cola.append((Ai, Aj))
        
    while cola != []:
      (Ai,Aj)= cola.pop(0)
      if remove_inconsistent_values(csp, Ai, Aj):
        if len(csp.domains[Ai]) == 0:
          return False
        for Ak in csp.get_neighbors(Ai):
          if Ak != Aj:
            cola.append((Ak, Ai))
    return True
        
def mrv(csp: DroneAssignmentCSP, assignment):
    sin_asignar= csp.get_unassigned_variables(assignment)
    minimo= math.inf
    for var in sin_asignar:
        if len(csp.domains[var]) < minimo:
            minimo= len(csp.domains[var])
            variable= var
    return variable
  
def lcv(csp: DroneAssignmentCSP, var, assignment):
    valores= csp.domains[var]
    ordenados= []
    retorno= []
    for valor in valores:
        conflictos= csp.get_num_conflicts(var, valor, assignment)
        ordenados.append((conflictos, valor))
    ordenados.sort()
    for e in ordenados:
        retorno.append(e[1])
    return retorno

def backtracking_ac3(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking search with AC-3 arc consistency.

    Tips:
    - AC-3 enforces arc consistency: for every pair of constrained variables (Xi, Xj), every value
      in Xi's domain must have at least one supporting value in Xj's domain.
    - Run AC-3 before starting backtracking to reduce domains globally.
    - After each assignment, run AC-3 on arcs involving the assigned variable's neighbors.
    - If AC-3 empties any domain, the current assignment is inconsistent - backtrack.
    - You can create helper functions such as:
      - a values_compatible function to check if two variable-value pairs are consistent with the constraints.
      - a revise function that removes unsupported values from one variable's domain.
      - an ac3 function that manages the queue of arcs to check and calls revise.
      - a backtrack function that integrates AC-3 into the search process.
    """
    # TODO: Implement your code here
    assignment= {}
    if not AC_3(csp):
        return None

    def recursiva_backtrack_ac3(assignment):
      if csp.is_complete(assignment):
       return assignment
      var= csp.get_unassigned_variables(assignment)[0]
      for value in csp.domains[var]:
        if csp.is_consistent(var, value, assignment):
          csp.assign(var, value, assignment)
          copia_dominios= {}
          for variable, lista in csp.domains.items():
            copia_dominios[variable]= lista.copy()
          arcos= []
          for neighbor in csp.get_neighbors(var):
              arcos.append((neighbor, var))
          if AC_3(csp, arcos):
              result= recursiva_backtrack_ac3(assignment)
              if result is not None:
                  return result
          csp.domains= copia_dominios
          csp.unassign(var, assignment)

      return None
    return recursiva_backtrack_ac3({})


def backtracking_mrv_lcv(csp: DroneAssignmentCSP) -> dict[str, str] | None:
    """
    Backtracking with Forward Checking + MRV + LCV.

    Tips:
    - Combine the techniques from backtracking_fc, mrv_heuristic, and lcv_heuristic.
    - MRV (Minimum Remaining Values): Select the unassigned variable with the fewest legal values.
      Tie-break by degree: prefer the variable with the most unassigned neighbors.
    - LCV (Least Constraining Value): When ordering values for a variable, prefer
      values that rule out the fewest choices for neighboring variables.
    - Use csp.get_num_conflicts(var, value, assignment) to count how many values would be ruled out for neighbors if var=value is assigned.
    """
    def recursiva_mrv_lcv(assignment):
        if csp.is_complete(assignment):
            return assignment
          
        var= mrv(csp, assignment)
        for value in lcv(csp, var, assignment):
            if csp.is_consistent(var, value, assignment):
                csp.assign(var, value, assignment)
                copia_dominios = {v: lista.copy() for v, lista in csp.domains.items()}

                dominios_ok = True
            for neighbor in csp.get_neighbors(var):
                if neighbor not in assignment:
                    csp.domains[neighbor] = [
                        v for v in csp.domains[neighbor]
                        if csp.is_consistent(neighbor, v, assignment)
                    ]
                    if not csp.domains[neighbor]:
                        dominios_ok = False
                        break

            if dominios_ok:
                result = recursiva_mrv_lcv(assignment)
                if result:
                    return result

            csp.domains = copia_dominios
            csp.unassign(var, assignment)
    return None