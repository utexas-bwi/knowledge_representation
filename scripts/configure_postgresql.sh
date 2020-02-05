#!/usr/bin/env bash

sudo service postgresql start

sudo -u postgres createdb knowledge_base
sudo -u postgres psql -d knowledge_base -f sql/schema_postgresql.sql
sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD 'nopass'"

if [[ -n "$IN_DOCKER" ]]; then
  echo "Executing Industrial CI setup"
  # The previous invocations won't work because the postgres user doesn't
  # have permission to access the working directory
  POSTGRES_HOME=/var/lib/postgresql
  cp sql/schema_postgresql.sql $POSTGRES_HOME
  sudo -u postgres psql -d knowledge_base -f $POSTGRES_HOME/schema_postgresql.sql
fi