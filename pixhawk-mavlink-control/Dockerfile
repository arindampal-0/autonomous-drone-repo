FROM python:3.7

RUN apt update && apt upgrade -y

WORKDIR /app

COPY requirements.txt requirements.txt

RUN pip install -r requirements.txt

ENTRYPOINT [ "bash" ]