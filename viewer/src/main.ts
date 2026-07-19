import './app.css';
import { mount } from 'svelte';
import App from './App.svelte';

const target = document.getElementById('app');
if (!target) throw new Error('#app not found');

// アクセス解析は Google Analytics (Consent Mode v2) に一本化。
// タグの読込・同意管理は src/analytics.ts が担当する (App の onMount で initAnalytics)。

export default mount(App, { target });
