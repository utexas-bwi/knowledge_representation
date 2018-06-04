#!/usr/bin/env sh

prepare_knowledge() {
    knowledge_rep_path=$(rospack find knowledge_representation)
    # Drop tables and establish schema
    mysql -u root -p -e "source $knowledge_rep_path/sql/create_database.sql"

    # Load arena prespecified knowledge
    xml_files_path=$(rospack find spr_qa)/../GPSRCmdGen/CommanFiles
    rosrun knowledge_representation populate_with_file $xml_files_path/Locations.xml $xml_files_path/Objects.xml

    # Load addenda
    rosrun knowledge_representation initialize_planner_test_configuration
}
