
import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import fr.uga.pddl4j.problem.operator.Effect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

import javax.xml.xpath.XPath;
import java.util.*;



/**
 * The class is an example. It shows how to create a simple A* search planner able to
 * solve an ADL problem by choosing the heuristic to used and its weight.
 *
 * @author D. Pellier
 * @version 4.0 - 30.11.2021
 */
@CommandLine.Command(name = "ASP",
        version = "ASP 1.0",
        description = "Solves a specified planning problem using A* search strategy.",
        sortOptions = false,
        mixinStandardHelpOptions = true,
        headerHeading = "Usage:%n",
        synopsisHeading = "%n",
        descriptionHeading = "%nDescription:%n%n",
        parameterListHeading = "%nParameters:%n",
        optionListHeading = "%nOptions:%n")
public class ASP extends AbstractPlanner {



    /**
     * The weight of the heuristic.
     */
    private double heuristicWeight;

    /**
     * The name of the heuristic used by the planner.
     */
    private StateHeuristic.Name heuristic;




    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
            paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
            description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
                    + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * Returns the name of the heuristic used by the planner to solve a planning problem.
     *
     * @return the name of the heuristic used by the planner to solve a planning problem.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }













    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(ASP.class.getName());





    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }






    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be instantiated.
     */


    @Override
    public boolean isSupported(Problem problem) {
        return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
                || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
                || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
                || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
                || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
                || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
                || problem.getRequirements().contains(RequireKey.FLUENTS)
                || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
                || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
                || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
                || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
                || problem.getRequirements().contains(RequireKey.PREFERENCES)
                || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
                || problem.getRequirements().contains(RequireKey.HIERARCHY))
                ? false : true;
    }




    /**
     * Search a solution plan for a planning problem using an A* search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     * @throws ProblemNotSupportedException if the problem to solve is not supported by the planner.
     */
    public Plan MTCS_PureRandomWalks(Problem problem) throws ProblemNotSupportedException, NullPointerException {
        // Maximum number of steps per walk
        final int MAX_STEPS = 4;

        // Number of iterations (random walks)
        final int NUM_ITERATIONS = 4;

        Random random = new Random();




        // Check if the problem is supported by the planner
        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        // First we create an instance of the heuristic to use to guide the search
        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);

        // We get the initial state from the planning problem
        final State init = new State(problem.getInitialState());





        // We create the root node of the tree search
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));


        // initial heuristic
        double hmin = heuristic.estimate(init, problem.getGoal());


        Plan plan = null;





        // We start the search


        Node current = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));
        List<Action> actions = problem.getActions();





        for (int iteration = 0; iteration < NUM_ITERATIONS; iteration++) {



            for (int step = 0; step < MAX_STEPS; step++) {
                // Check if the goal is satisfied
                if (current.satisfy(problem.getGoal())) {
                   plan = extractPlan(current, problem); // Call extractPlan with the final node



                }

                // Get all applicable actions
                List<Action> applicableActions = new ArrayList<>();
                for (Action action : actions) {
                    if (action.isApplicable(current)) {
                        applicableActions.add(action);
                    }
                }

                // If no actions are applicable, break the loop
                if (applicableActions.isEmpty()) {

                    break;

                }

                // Randomly select an action
                int r = random.nextInt(applicableActions.size()) ;


                Action selectedAction = applicableActions.get(random.nextInt(applicableActions.size()));

                // Create the next node by applying the action's effects
                Node next = new Node(current);
                for (ConditionalEffect ce : selectedAction.getConditionalEffects()) {
                    if (current.satisfy(ce.getCondition())) {
                        next.apply(ce.getEffect());


                    }
                    final double g = current.getCost() + 1;

                    next.setCost(g);
                    next.setParent(current);
                    next.setAction(step);
                    next.setHeuristic(heuristic.estimate(current, problem.getGoal()));


                    if (heuristic.estimate(next, problem.getGoal()) < hmin){

                        hmin = heuristic.estimate(next, problem.getGoal());

                        current= next ;


                    }





                }





            }
        }

        return plan;






    }










    /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */


    private Plan extractPlan(final Node node, final Problem problem) throws  NullPointerException{
        final Plan plan = new SequentialPlan();
        try {

            Node n = node;


            //  final Plan plan = new SequentialPlan();
            while (n.getAction() != -1) {
                final Action a = problem.getActions().get(n.getAction());
                plan.add(0, a);
                n = n.getParent();
            }} catch (NullPointerException e){
            LOGGER.error("Null pointer exception in extractPlan *********", e);




        }

        return plan;
    }








    /**
     * Search a solution plan to a specified domain and problem using A*.
     *
     * @param problem the problem to solve.
     * @return the plan found or null if no plan was found.
     */
    @Override
    public Plan solve(final Problem problem)  throws ProblemNotSupportedException  {



        LOGGER.info("* Starting A* search with MTCS \n");
        // Search a solution



        final long begin = System.currentTimeMillis();
        Plan plan = this.MTCS_PureRandomWalks(problem);
        //final Plan plan = this.astar(problem);
        final long end = System.currentTimeMillis();




        // If a plan is found update the statistics of the planner
        // and log search information
        if (plan != null) {
            LOGGER.info("* A* search succeeded with MTCS pure random walks\n");
            this.getStatistics().setTimeToSearch(end - begin);
        } else {
            LOGGER.info("* A* search failed\n");
            LOGGER.info("* A* the plane is empty \n");
            System.out.println("****");
        }
        // Return the plan found or null if the search fails.
        return plan;


    }


























    /**
     * The main method of the <code>ASP</code> planner.
     *
     * @param args the arguments of the command line.
     */
    public static void main(String[] args)  {
        try {
            final ASP planner = new ASP();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }


    /**
     * This class implements a node of the tree search.
     *
     * @author D. Pellier
     * @version 1.0 - 02.12.2021
     */
    public final class Node extends State {

        /**
         * The parent node of this node.
         */
        private Node parent;

        /**
         * The action apply to reach this node.
         */
        private int action;

        /**
         * The cost to reach this node from the root node.
         */
        private double cost;

        /**
         * The estimated distance to the goal from this node.
         */
        private double heuristic;

        /**
         * The depth of the node.
         */
        private int depth;

        /**
         * Creates a new node from a specified state.
         *
         * @param state the state.
         */
        public Node(State state) {
            super(state);
        }

        /**
         * Creates a new node with a specified state, parent node, operator,
         * cost and heuristic value.
         *
         * @param state     the logical state of the node.
         * @param parent    the parent node of the node.
         * @param action   the action applied to reached the node from its parent.
         * @param cost      the cost to reach the node from the root node.
         * @param heuristic the estimated distance to reach the goal from the node.
         */
        public Node(State state, Node parent, int action, double cost, double heuristic) {
            super(state);
            this.parent = parent;
            this.action = action;
            this.cost = cost;
            this.heuristic = heuristic;
            this.depth = -1;
        }

        /**
         * Creates a new node with a specified state, parent node, operator, cost,
         * depth and heuristic value.
         *
         * @param state     the logical state of the node.
         * @param parent    the parent node of the node.
         * @param action    the action applied to reached the node from its parent.
         * @param cost      the cost to reach the node from the root node.
         * @param depth     the depth of the node.
         * @param heuristic the estimated distance to reach the goal from the node.
         */
        public Node(State state, Node parent, int action, double cost, int depth, double heuristic) {
            super(state);
            this.parent = parent;
            this.action = action;
            this.cost = cost;
            this.depth = depth;
            this.heuristic = heuristic;
        }

        /**
         * Returns the action applied to reach the node.
         *
         * @return the action applied to reach the node.
         */
        public final int getAction() {
            return this.action;
        }

        /**
         * Sets the action applied to reach the node.
         *
         * @param action the action to set.
         */
        public final void setAction(final int action) {
            this.action = action;
        }

        /**
         * Returns the parent node of the node.
         *
         * @return the parent node.
         */
        public final Node getParent() {
            return parent;
        }

        /**
         * Sets the parent node of the node.
         *
         * @param parent the parent to set.
         */
        public final void setParent(Node parent) {
            this.parent = parent;
        }

        /**
         * Returns the cost to reach the node from the root node.
         *
         * @return the cost to reach the node from the root node.
         */
        public final double getCost() {
            return cost;
        }

        /**
         * Sets the cost needed to reach the node from the root node.
         *
         * @param cost the cost needed to reach the node from the root nod to set.
         */
        public final void setCost(double cost) {
            this.cost = cost;
        }

        /**
         * Returns the estimated distance to the goal from the node.
         *
         * @return the estimated distance to the goal from the node.
         */
        public final double getHeuristic() {
            return heuristic;
        }

        /**
         * Sets the estimated distance to the goal from the node.
         *
         * @param estimates the estimated distance to the goal from the node to set.
         */
        public final void setHeuristic(double estimates) {
            this.heuristic = estimates;
        }

        /**
         * Returns the depth of this node.
         *
         * @return the depth of this node.
         */
        public int getDepth() {
            return this.depth;
        }

        /**
         * Set the depth of this node.
         *
         * @param depth the depth of this node.
         */
        public void setDepth(final int depth) {
            this.depth = depth;
        }

        /**
         * Returns the value of the heuristic function, i.e.,
         * <code>this.node.getCost() + this.node.getHeuristic()</code>.
         *
         * @param weight the weight of the heuristic.
         * @return the value of the heuristic function, i.e.,
         * <code>this.node.getCost() + this.node.getHeuristic()</code>.
         */
        public final double getValueF(double weight) {
            return weight * this.heuristic + this.cost;
        }

    }





















}

















