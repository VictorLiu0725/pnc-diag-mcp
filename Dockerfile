FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY pyproject.toml README.md LICENSE NOTICE THIRD_PARTY_NOTICES.md ./
COPY configs ./configs
COPY src ./src
COPY fault_sim_kit ./fault_sim_kit

RUN pip install --no-cache-dir . \
    && useradd -m -u 1000 mcpuser \
    && chown -R mcpuser:mcpuser /app

USER mcpuser

ENTRYPOINT ["pnc-diag-mcp"]
