package org.sbpo2025.challenge;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import org.apache.commons.lang3.time.StopWatch;

import java.util.*;
import java.util.concurrent.TimeUnit;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // 10 minutos em milissegundos

    // Dados do problema conforme descrição do desafio
    protected List<Map<Integer, Integer>> orders;   // Lista de pedidos
    protected List<Map<Integer, Integer>> aisles;   // Lista de corredores
    protected int nItems;                           // Número total de itens
    protected int waveSizeLB;                       // Limite inferior do tamanho da wave (LB)
    protected int waveSizeUB;                       // Limite superior do tamanho da wave (UB)

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles,
            int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        try {
            IloCplex cplex = new IloCplex();
            cplex.setParam(IloCplex.Param.TimeLimit, getRemainingTime(stopWatch));

            // ---------------------------------------------------------------
            // Variáveis de decisão
            // ---------------------------------------------------------------
            // x_o = 1 se o pedido o está na wave
            IloIntVar[] x = new IloIntVar[orders.size()];

            // y_a = 1 se o corredor a é utilizado
            IloIntVar[] y = new IloIntVar[aisles.size()];

            for (int i = 0; i < orders.size(); i++) {
                x[i] = cplex.boolVar("x_" + i);
            }

            for (int j = 0; j < aisles.size(); j++) {
                y[j] = cplex.boolVar("y_" + j);
            }

            // ---------------------------------------------------------------
            // Função Objetivo Linearizada
            // ---------------------------------------------------------------
            // Objetivo original: max (Σu_oi * x_o) / (Σy_a)
            // Estratégia: Maximizar Σu_oi * x_o - α * Σy_a (α = penalidade por corredor)
            IloLinearNumExpr obj = cplex.linearNumExpr();
            for (int o = 0; o < orders.size(); o++) {
                // Soma das unidades do pedido o (Σu_oi)
                int units = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
                obj.addTerm(units, x[o]);
            }
            for (int a = 0; a < aisles.size(); a++) {
                // Penalização por uso de corredores (α = 1000)
                obj.addTerm(-1000, y[a]);
            }
            cplex.addMaximize(obj);

            // ---------------------------------------------------------------
            // Restrições
            // ---------------------------------------------------------------
            // Restrição 3.5.1 e 3.5.2: LB ≤ Σu_oi * x_o ≤ UB
            IloLinearNumExpr totalUnitsExpr = cplex.linearNumExpr();
            for (int o = 0; o < orders.size(); o++) {
                int units = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
                totalUnitsExpr.addTerm(units, x[o]);
            }
            cplex.addGe(totalUnitsExpr, waveSizeLB); // LB
            cplex.addLe(totalUnitsExpr, waveSizeUB); // UB

            // Restrição 3.5.3: Σu_oi * x_o ≤ Σu_ai * y_a, ∀i ∈ I (Eq. 4)
            for (int item = 0; item < nItems; item++) {
                IloLinearNumExpr demanda = cplex.linearNumExpr(); // Demanda do item
                IloLinearNumExpr oferta = cplex.linearNumExpr();  // Oferta do item

                for (int o = 0; o < orders.size(); o++) {
                    if (orders.get(o).containsKey(item)) {
                        demanda.addTerm(orders.get(o).get(item), x[o]);
                    }
                }

                for (int a = 0; a < aisles.size(); a++) {
                    if (aisles.get(a).containsKey(item)) {
                        oferta.addTerm(aisles.get(a).get(item), y[a]);
                    }
                }

                cplex.addLe(demanda, oferta); // Demanda ≤ Oferta
            }

            // Restrição 3.5.4: Σy_a ≥ 1 (Pelo menos um corredor)
            cplex.addGe(cplex.sum(y), 1);

            // ---------------------------------------------------------------
            // Resolução e Recuperação da Solução
            // ---------------------------------------------------------------
            if (cplex.solve()) {
                Set<Integer> selectedOrders = new HashSet<>();
                for (int o = 0; o < orders.size(); o++) {
                    if (cplex.getValue(x[o]) > 0.9) { // Tolerância para valores binários
                        selectedOrders.add(o);
                    }
                }

                Set<Integer> selectedAisles = new HashSet<>();
                for (int a = 0; a < aisles.size(); a++) {
                    if (cplex.getValue(y[a]) > 0.9) {
                        selectedAisles.add(a);
                    }
                }

                return new ChallengeSolution(selectedOrders, selectedAisles);
            }

            cplex.end();
        } catch (IloException e) {
            e.printStackTrace();
        }
        return null;
    }

    /*
     * Get the remaining time in seconds
     */
    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0);
    }

    protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return false;
        }

        int[] totalUnitsPicked = new int[nItems];
        int[] totalUnitsAvailable = new int[nItems];

        // Calculate total units picked
        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        // Calculate total units available
        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        // Check if the total units picked are within bounds
        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        // Check if the units picked do not exceed the units available
        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
                return false;
            }
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }
        int totalUnitsPicked = 0;

        // Calculate total units picked
        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream()
                    .mapToInt(Integer::intValue)
                    .sum();
        }

        // Calculate the number of visited aisles
        int numVisitedAisles = visitedAisles.size();

        // Objective function: total units picked / number of visited aisles
        return (double) totalUnitsPicked / numVisitedAisles;
    }
}
