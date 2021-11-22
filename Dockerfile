FROM python:3.6
RUN apt-get update
RUN mkdir gakachu
WORKDIR /gakachu
COPY requirements.txt /gakachu/requirements.txt
RUN pip install -r requirements.txt
COPY src/main.py /gakachu/src/main.py
COPY dist/ /gakachu/dist/
EXPOSE 5000
CMD python3 src/main.py