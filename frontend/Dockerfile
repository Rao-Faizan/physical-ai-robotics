FROM node:20-alpine

WORKDIR /app

# Install dependencies
COPY package.json package-lock.json ./
RUN npm install

# Copy application
COPY . .

# Expose port
EXPOSE 3000

# Start application
CMD ["npm", "start", "--", "--host", "0.0.0.0"]
