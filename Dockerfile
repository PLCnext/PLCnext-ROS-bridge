FROM busybox@sha256:44f3ac403c0c660b92477033aa7083bda9145a70bd7059f409e54ccc2f2f3a81
ENV PORT=8000
LABEL maintainer="Luhmann <mluhmann@phoenixcontact.com>"

ADD index.html index.html

EXPOSE $PORT

COPY start.sh start.sh
RUN chmod -v +x start.sh

HEALTHCHECK CMD nc -z localhost $PORT
CMD ["./start.sh"]