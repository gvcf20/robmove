"""
sim/test_connection.py
Etapa 1: testa a conexao Python <-> CoppeliaSim via ZMQ Remote API.
ABRA o CoppeliaSim (cena vazia) ANTES de rodar este script.
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main():
    print("conectando ao CoppeliaSim em localhost:23000 ...")
    client = RemoteAPIClient()            # host/porta padrao
    sim = client.require('sim')
    ver = sim.getInt32Param(sim.intparam_program_version)
    print(f"CONECTADO! versao do CoppeliaSim: {ver}")

    # cria um cubo na origem so p/ confirmar que conseguimos modificar a cena
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.3, 0.3, 0.3], 0)
    sim.setObjectAlias(h, "teste_cubo")
    print("criei um cubo 'teste_cubo' na origem -> olhe a cena no CoppeliaSim.")

    input("\nENTER aqui no terminal p/ remover o cubo e finalizar...")
    sim.removeObjects([h])
    print("removido. conexao OK.")


if __name__ == "__main__":
    main()
