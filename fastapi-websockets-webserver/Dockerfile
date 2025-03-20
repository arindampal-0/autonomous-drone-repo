FROM python:3.7

RUN apt update && apt upgrade -y

RUN pip install --upgrade pip

WORKDIR /app

COPY requirements.txt requirements.txt

RUN pip install -r requirements.txt

ENTRYPOINT [ "bash" ]