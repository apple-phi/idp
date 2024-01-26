import ast
import collections
import itertools
import numpy as np
import networkx as nx

# Use N=-S and E=-W to make direction inverting easier
N, E, S, W = 0, 1, 2, 3
opposites = {N: S, E: W, S: N, W: E}

# Network in the format (node1, node2) : (weight, direction)
# sorted by key
# where node1 < node2
# and direction is from node1 to node2
nodes = {
    (0, 3): (1, N),
    (1, 6): (1, N),
    (2, 7): (1, N),
    (3, 4): (1, E),
    (3, 8): (2, N),
    (4, 5): (1, N),
    (4, 6): (1, E),
    (6, 7): (2, E),
    (7, 12): (2, N),
    (8, 9): (2, E),
    # (8, 18): (1, N),
    # (15, 18): (2, W),
    (9, 10): (1, E),
    (9, 13): (1, N),
    (10, 11): (1, S),
    (10, 12): (1, E),
    # (12, 19): (1, N),
    # (16, 19): (2, E),
    (13, 14): (1, W),
    (13, 15): (1, N),
    (15, 16): (1, E),
    (16, 17): (1, S),
}

# Convert to dict-of-dicts format for networkx
dod = collections.defaultdict(lambda: collections.defaultdict(dict))
for (i, j), (w, d) in nodes.items():
    assert i < j
    assert w > 0
    dod[i][j]["weight"] = w
    dod[i][j]["dir"] = d

G = nx.Graph(dod)


def arr_of_arr_to_cpp(arr: np.ndarray):
    assert arr.ndim == 2
    cpp_code = (
        str(arr)
        .replace("[", "{")
        .replace("]", "}")
        .replace(" ", ",")
        .replace("\n,", ",\n")
    )
    while ",," in cpp_code:
        cpp_code = cpp_code.replace(",,", ",")
    cpp_code = cpp_code.replace("{,", "{")
    try:
        type(ast.literal_eval(cpp_code)) == set, "Invalid C++ code generated"
    except TypeError as e:
        return cpp_code
    except SyntaxError as e:
        raise SyntaxError("Invalid C++ code generated") from e
    raise RuntimeError("Something went wrong in C++ code generation")


def gen_dir_mat():
    """Generate a "direction matrix" as a C++ vector<vector<int>>
    Given two nodes i and j
    the direction matrix will contain the direction from i to j
    along the shortest path.
    """
    sentinel = -1
    # The direction matrix has the same shape as the adjacency matrix
    arr = np.full_like(nx.adjacency_matrix(G).todense(), fill_value=sentinel)
    for i, j in itertools.product(G.nodes, G.nodes):
        if i == j:
            continue
        target = nx.shortest_path(G, i, j)[1]  # 1st edge in shortest path
        arr[i, j] = G.edges[i, target]["dir"]
        if i>target:
            arr[i, j] = opposites[arr[i, j]]
    assert np.all(np.diag(arr) == sentinel)
    assert collections.Counter(arr.flatten())[sentinel] == len(np.diag(arr))
    return arr


def gen_navigation_matrix():
    """Generate a mapping from (node, direction) to next node."""
    pairs = set()
    for i, j in itertools.product(G.nodes, G.nodes):
        if i == j:
            continue
        target = nx.shortest_path(G, i, j)[1]
        d = G.edges[i, target]["dir"]
        if i > target:
            d = opposites[d]
        pairs.add(
            "{{"
            + f"{i}, {d}"
            + "}"
            + f", {target}"
            + "},"
        )
    return (
        "arx::map<arx::pair<int, int>, int> navigation_map = {\n"
        + "\n".join(pairs)
        + "\n};"
    )


def draw_graph():
    import matplotlib.pyplot as plt

    pos = nx.spring_layout(G)
    nx.draw(G, pos)
    nx.draw_networkx_labels(G, pos)
    edge_labels = nx.get_edge_attributes(G, "weight")
    nx.draw_networkx_edge_labels(G, pos, edge_labels)
    plt.show()


if __name__ == "__main__":
    print(arr_of_arr_to_cpp(gen_dir_mat()))
    print(gen_navigation_matrix())
