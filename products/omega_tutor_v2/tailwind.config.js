/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        bg: {
          primary: "#fafafa",
          secondary: "#f5f5f5",
          tertiary: "#ebebeb",
        },
        text: {
          primary: "#1a1a1a",
          secondary: "#4a4a4a",
          muted: "#7a7a7a",
        },
        accent: "#2563eb",
        success: "#059669",
        warning: "#d97706",
        error: "#dc2626",
      },
      fontFamily: {
        sans: ["Inter", "system-ui", "sans-serif"],
      },
    },
  },
  plugins: [],
};
