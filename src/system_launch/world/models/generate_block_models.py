#!/usr/bin/env python

alphabet = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"]

dae_file_text = ""
config_file_text = ""
sdf_file_text = ""

for letter in alphabet[1:]:
    model_name = "block_" + letter
    name = "Block " + letter
    letter_name = "letter " + letter

    # copy dae file
    with open("./block_A/block_A.dae", 'r') as dae_src_file:
        dae_file_text = dae_src_file.read().replace("block_A", model_name)

    dae_file_path = "./" + model_name + "/" + model_name + ".dae"
    with open(dae_file_path, 'w') as dae_dest_file:
        dae_dest_file.write(dae_file_text)

    # copy model config
    with open("./block_A/model.config", 'r') as config_src_file:
        config_file_text = config_src_file.read().replace("Block A", name).replace("letter A", letter_name)

    config_file_path = "./" + model_name + "/model.config"
    with open(config_file_path, 'w') as config_dest_file:
        config_dest_file.write(config_file_text)

    # copy model sdf
    with open("./block_A/model.sdf", 'r') as sdf_src_file:
        sdf_file_text = sdf_src_file.read().replace("block_A", model_name)

    sdf_file_path = "./" + model_name + "/model.sdf"
    with open(sdf_file_path, 'w') as sdf_dest_file:
        sdf_dest_file.write(sdf_file_text)


        
